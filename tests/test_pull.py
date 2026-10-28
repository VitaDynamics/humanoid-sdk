"""Opt-in pull state/snapshot/session lifecycle; no network or motor IO."""
import threading
import unittest
import time
from collections import deque
from dataclasses import replace
from types import SimpleNamespace as S
from unittest.mock import patch
from locomotion_aorta import PullController,PullStateSource,PullTransport,ExternalControlTimeoutError
from locomotion_aorta import ControlAck,ControlStatus,ControlOperation,ExternalPhase
from locomotion_aorta import BodyCmd,BodyJointCmd,ControllerClosedError,NotRunningError
from locomotion_aorta.transport import BODY_CMD_TOPIC,BODY_STATE_TOPIC,CONTROL_STATUS_TOPIC,CONTROL_REQUEST_TOPIC
from fake_transport import FakeTransport
from locomotion_aorta.body_types import DescriptorRef

class Ring:
    def __init__(self):self.samples=deque();self.waits=[];self.closed=False
    def recv(self,timeout):self.waits.append(timeout);return self.samples.popleft() if self.samples else None
    def try_recv(self):return self.recv(0)
    def close(self):self.closed=True

class PullTest(unittest.TestCase):
    def test_teleop_pull_entry_and_body_rejection_keep_original_lock_order(self):
        c, t = self.fixture(action='HUMANOID_TELEOP')
        original = t.poll_control_status
        polls = []

        def poll():
            self.assertFalse(c._condition._is_owned())
            polls.append(True)
            original()

        with patch.object(t, 'poll_control_status', side_effect=poll):
            active = c.enter_control('HUMANOID_TELEOP', .2)
            before = list(t.published)
            with self.assertRaises(NotRunningError):
                c.publish(BodyCmd(tuple(BodyJointCmd() for _ in range(31))))
            self.assertEqual(t.published, before)
            self.assertEqual(c._next_external_sequence, 1)
            self.assertEqual(c._control_context().generation, active.generation)
            self.assertEqual(c._next_external_sequence, 1)
            self.assertEqual(c._reserve_command().sequence, 1)
            self.assertEqual(c.exit_control(.2).phase, ExternalPhase.IDLE)
        self.assertTrue(polls)
        self.assertEqual([r.action_name for t, r in t.published], ['HUMANOID_TELEOP'] * 3)

    def test_teleop_pull_foreign_action_or_cancel_rejects_ee_context(self):
        c, t = self.fixture(action='HUMANOID_TELEOP')
        owned = c.enter_control('HUMANOID_TELEOP', .2)
        t.emit(CONTROL_STATUS_TOPIC, replace(owned, status_seq=2, current_action='EXTERNAL'))
        with self.assertRaises(NotRunningError):
            c._reserve_command()
        with self.assertRaises(NotRunningError):
            c._control_context()
        self.assertEqual(c._next_external_sequence, 1)
        t.emit(CONTROL_STATUS_TOPIC, replace(owned, status_seq=3))
        self.assertEqual(c._reserve_command().sequence, 1)
        c._best_effort_cancel()
        # An already queued RUNNING does not reopen locally cancelled control.
        t.emit(CONTROL_STATUS_TOPIC, replace(owned, status_seq=4))
        with self.assertRaises(NotRunningError):
            c._reserve_command()
        with self.assertRaises(NotRunningError):
            c._control_context()
        self.assertEqual(c._next_external_sequence, 2)

    def test_rpc_context_drains_outside_shared_lock_and_preserves_motion_sequence(self):
        c,t=self.fixture();owned=c.enter_external(.2)
        original=t.poll_control_status
        def poll():
            self.assertFalse(c._condition._is_owned())
            original()
        with patch.object(t,'poll_control_status',side_effect=poll):
            before=list(t.published)
            self.assertEqual(c._control_context().generation,42)
            self.assertEqual(t.published,before)
            self.assertEqual(c._next_external_sequence,1)
            for i,changed in enumerate(({'client_id':17},{'control_id':19}),2):
                t.emit(CONTROL_STATUS_TOPIC,replace(owned,status_seq=i,**changed))
                with self.assertRaises(NotRunningError):c._control_context()
                self.assertEqual(c._next_external_sequence,1)
                self.assertEqual(t.published,before)

    def test_rpc_context_poll_failure_or_close_during_poll_never_returns_old_permission(self):
        c,t=self.fixture();c.enter_external(.2)
        before=list(t.published)
        with patch.object(t,'poll_control_status',side_effect=OSError('drain failed')) as poll:
            with self.assertRaisesRegex(OSError,'drain failed'):c._control_context()
            self.assertEqual(poll.call_count,1)
        self.assertEqual(t.published,before)
        with patch.object(t,'poll_control_status',side_effect=c.close):
            with self.assertRaises(ControllerClosedError):c._control_context()
        sent_at_close=list(t.published)
        with patch.object(t,'poll_control_status') as poll:
            with self.assertRaises(ControllerClosedError):c._control_context()
            poll.assert_not_called()
        self.assertEqual(t.published,sent_at_close)

    def transport(self):
        t=PullTransport.__new__(PullTransport);t._closed=False;t._state_lock=threading.Lock();t._status_lock=threading.RLock();t._status=None;t._latest_state=None;t._latest_sample=None;t._latest_body_state=None
        t._pulls={BODY_STATE_TOPIC:Ring(),CONTROL_STATUS_TOPIC:Ring()};t._callbacks={BODY_STATE_TOPIC:[],CONTROL_STATUS_TOPIC:[]}
        t._bindings=S(control_status_decoder=lambda v:v);t._subscriber_spec=lambda topic:(None,lambda v:v,None)
        # These original transport-only tests exercise receive mechanics; the
        # connection-bound tests below install the real identity gate instead.
        t._candidate_gate=lambda candidate, commit:(commit() or True)
        return t

    def test_constructor_no_publishers_two_native_depth1_rings(self):
        calls=[]
        class Node:
            def __init__(self,*a):self.closed=False
            def group(self):return 'default'
            def create_publisher_typed(self,*a,**k):raise AssertionError('publisher')
            def create_subscriber_typed(self,*a,**k):raise AssertionError('callback')
            def create_subscriber_pull(self,topic,**kw):calls.append((topic,kw));return Ring()
            def close(self):self.closed=True
        bindings=S(aorta=S(Node=Node,QoS=S(sensor_data=lambda:'sensor',state_update=lambda:'status')))
        with patch('locomotion_aorta.transport._load_aorta_bindings',return_value=bindings):
            t=PullTransport(node_name='test',group='default',publishers=False)
            self.assertEqual(t.publishers,{});self.assertEqual(len(calls),2)
            self.assertTrue(all(kw['depth']==1 and kw['options'].include_receive_timestamp for _,kw in calls))
            t.close();self.assertTrue(t.node.closed)

    def test_status_latest_snapshot_caller_thread_and_unsubscribe(self):
        t=self.transport();seen=[];threads=[]
        sub=t.subscribe(CONTROL_STATUS_TOPIC,lambda s:(seen.append(s),threads.append(threading.get_ident())))
        s=S(status_seq=99);t._pulls[CONTROL_STATUS_TOPIC].samples.append(S(payload=s))
        self.assertIs(t.control_status,s);self.assertIs(t.control_status,s)
        self.assertEqual(seen,[s]);self.assertEqual(threads,[threading.get_ident()])
        sub.close();sub.close();self.assertFalse(t._callbacks[CONTROL_STATUS_TOPIC])

    def test_wait_state_timestamp_after_recv_and_native_arrays(self):
        t=self.transport();t._pulls[BODY_STATE_TOPIC].samples.append(S(payload=b'state',context=S(receive_stamp_ns=42)))
        from dataclasses import dataclass
        @dataclass(frozen=True)
        class State:
            received_monotonic_ns:int=0
            received_realtime_ns:int=0
            pull_context_receive_stamp_ns:int=0
        with patch('locomotion_aorta.pull.decode_body_state_arrays',return_value=State()):
            result=t.wait_state(.1)
        self.assertGreater(result.received_monotonic_ns,0);self.assertGreater(result.received_realtime_ns,0)
        self.assertEqual(result.pull_context_receive_stamp_ns,42)

    def test_timeout_invalid_timeout_and_closed_never_return_old_state(self):
        t=self.transport()
        with self.assertRaises(TimeoutError):t.wait_state(.01)
        with self.assertRaises(TimeoutError):t.wait_state(0)
        for timeout in (-1,True,False,float('nan'),'0',0j):
            with self.assertRaises(ValueError):t.wait_state(timeout)
        t._closed=True
        with self.assertRaises(RuntimeError):t.wait_state(.01)
        with self.assertRaises(RuntimeError):t.poll_control_status()

    def test_latest_pending_replaces_cache_and_empty_ring_keeps_original_receipt(self):
        t=self.transport()
        from dataclasses import dataclass
        @dataclass(frozen=True)
        class State:
            state_id:int
            received_monotonic_ns:int=0
            received_realtime_ns:int=0
            pull_context_receive_stamp_ns:int=0
        ring=t._pulls[BODY_STATE_TOPIC]
        ring.samples.append(S(payload=11,context=S(receive_stamp_ns=42)))
        with patch('locomotion_aorta.pull.decode_body_state_arrays',side_effect=lambda value:State(value)) as decode:
            first=t.wait_state(0)
            self.assertIs(t.wait_state(0),first)
            self.assertEqual(decode.call_count,1)
            ring.samples.append(S(payload=19,context=S(receive_stamp_ns=43)))
            latest=t.wait_state(.1)
            self.assertEqual(latest.state_id,19)
            self.assertIs(t.wait_state(.1),latest)
            self.assertEqual(decode.call_count,2)
            self.assertNotEqual(first.state_id,latest.state_id)
            def close_while_polling():t._closed=True;return None
            ring.try_recv=close_while_polling
            with self.assertRaises(RuntimeError):t.wait_state(0)

    def test_close_during_recv_unblocks_and_never_returns_frame_after_close(self):
        t=self.transport();entered=threading.Event();closed=threading.Event();errors=[]
        class Blocking:
            def try_recv(self):return None
            def recv(self,timeout):entered.set();closed.wait(timeout);return None
        t._pulls[BODY_STATE_TOPIC]=Blocking()
        def worker():
            try:t.wait_state(.5)
            except BaseException as e:errors.append(e)
        thread=threading.Thread(target=worker);thread.start();self.assertTrue(entered.wait(.2))
        t._closed=True;closed.set();thread.join(.5)
        self.assertFalse(thread.is_alive());self.assertIsInstance(errors[0],RuntimeError)

    def fixture(self,missing_status=False,reject=False,action='EXTERNAL'):
        t=FakeTransport();pending=[];original=t.emit
        def emit(topic,message):
            if topic==CONTROL_STATUS_TOPIC:pending.append(message)
            else:original(topic,message)
        t.emit=emit
        def poll():
            while pending:original(CONTROL_STATUS_TOPIC,pending.pop(0))
        t.poll_control_status=poll
        t._wait_body_state=lambda timeout:t.wait_state(timeout).body_state()
        def publish(topic,req):
            if topic!=CONTROL_REQUEST_TOPIC:return
            t.emit('/sdk/foot_humanoid/control/ack',ControlAck(request_id=req.request_id,operation=req.operation,accepted=not reject,client_id=9,control_id=13,descriptor_ref=DescriptorRef(11)))
            if not missing_status and req.operation in(ControlOperation.COMMIT,ControlOperation.CANCEL):
                cancel=req.operation==ControlOperation.CANCEL
                t.emit(CONTROL_STATUS_TOPIC,ControlStatus(status_seq=2 if cancel else 1,current_action='PASSIVE'if cancel else action,phase=ExternalPhase.IDLE if cancel else ExternalPhase.RUNNING,client_id=9,control_id=13,generation=42,descriptor_ref=DescriptorRef(11)))
        t.on_publish=publish;c=PullController(client_id=7,_transport=t);c.get_robot_info(.1);self.addCleanup(c.close);return c,t

    def test_enter_cancel_original_ack_lease_generation_no_body_callback(self):
        c,t=self.fixture();self.assertNotIn(BODY_STATE_TOPIC,t.callbacks)
        self.assertEqual(c.enter_external(.2).generation,42)
        self.assertEqual(c.exit_external(.2).current_action,'PASSIVE')
        self.assertEqual([r.operation for _,r in t.published],[ControlOperation.PREPARE,ControlOperation.COMMIT,ControlOperation.CANCEL])

    def test_reservation_drains_pending_generation_and_revocation(self):
        c,t=self.fixture()
        c.enter_external(.2)
        command=BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31)))
        first=c.publish(command)
        t.emit(CONTROL_STATUS_TOPIC,ControlStatus(
            status_seq=2,current_action='EXTERNAL',phase=ExternalPhase.RUNNING,
            client_id=9,control_id=13,generation=84,descriptor_ref=DescriptorRef(11)))
        second=c.publish(command)
        self.assertEqual((first.session_id,first.sequence),(42,1))
        self.assertEqual((second.session_id,second.sequence),(84,1))
        t.emit(CONTROL_STATUS_TOPIC,ControlStatus(
            status_seq=3,current_action='PASSIVE',phase=ExternalPhase.IDLE,
            client_id=9,control_id=13,generation=84,descriptor_ref=DescriptorRef(11)))
        with self.assertRaises(NotRunningError):c.publish(command)
        self.assertEqual([m for topic,m in t.published if topic==BODY_CMD_TOPIC],[first,second])
        t.emit(CONTROL_STATUS_TOPIC,ControlStatus(
            status_seq=4,current_action='EXTERNAL',phase=ExternalPhase.RUNNING,
            client_id=9,control_id=13,generation=84,descriptor_ref=DescriptorRef(11)))
        resumed=c.publish(command)
        self.assertEqual((resumed.session_id,resumed.sequence),(84,2))

    def test_closed_controller_never_drains_pending_status_or_publishes(self):
        controller, transport = self.fixture()
        owned = controller.enter_external(.2)
        controller.close()
        sent_at_close = list(transport.published)
        transport.emit(CONTROL_STATUS_TOPIC, replace(
            owned, status_seq=2, generation=84))
        command = BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31)))

        with patch.object(transport, 'poll_control_status',
                          wraps=transport.poll_control_status) as poll:
            with self.assertRaises(ControllerClosedError):
                controller._reserve_command()
            with self.assertRaises(ControllerClosedError):
                controller.publish(command)
            self.assertIs(controller.status, owned)
            poll.assert_not_called()

        self.assertEqual(transport.published, sent_at_close)
        self.assertEqual(transport.close_count, 1)

    def test_pending_foreign_owner_or_lease_cannot_consume_key(self):
        for phase in (ExternalPhase.RUNNING, ExternalPhase.TIMEOUT_HOLD):
            for foreign in ({'client_id': 17},
                            {'control_id': 19}):
                with self.subTest(phase=phase, foreign=foreign):
                    controller, transport = self.fixture()
                    owned = controller.enter_external(.2)
                    command = BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31)))
                    first = controller.publish(command)
                    requests = [message for topic, message in transport.published
                                if topic == CONTROL_REQUEST_TOPIC]
                    transport.emit(CONTROL_STATUS_TOPIC, replace(
                        owned, status_seq=2, phase=phase,
                        generation=84 if phase == ExternalPhase.RUNNING else 42,
                        **foreign))

                    with self.assertRaises(NotRunningError):
                        controller._reserve_command()
                    with self.assertRaises(NotRunningError):
                        controller.publish(command)
                    self.assertEqual(
                        [message for topic, message in transport.published
                         if topic == BODY_CMD_TOPIC], [first])
                    self.assertEqual(
                        [message for topic, message in transport.published
                         if topic == CONTROL_REQUEST_TOPIC], requests)

                    transport.emit(CONTROL_STATUS_TOPIC, replace(owned, status_seq=3))
                    resumed = controller.publish(command)
                    self.assertEqual((resumed.session_id, resumed.sequence), (42, 2))

    def test_pending_invalid_generation_does_not_reset_or_consume_key(self):
        for generation in (0, -1, True, False, 42.0, '42', 1 << 64):
            with self.subTest(generation=generation):
                controller, transport = self.fixture()
                owned = controller.enter_external(.2)
                command = BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31)))
                first = controller.publish(command)
                transport.emit(CONTROL_STATUS_TOPIC, replace(
                    owned, status_seq=2, generation=generation))

                with self.assertRaises(NotRunningError):
                    controller.publish(command)
                self.assertEqual(
                    [message for topic, message in transport.published
                     if topic == BODY_CMD_TOPIC], [first])

                transport.emit(CONTROL_STATUS_TOPIC, replace(owned, status_seq=3))
                resumed = controller.publish(command)
                self.assertEqual((resumed.session_id, resumed.sequence), (42, 2))

    def test_status_poll_failure_does_not_send_retry_cancel_or_consume_key(self):
        controller, transport = self.fixture()
        controller.enter_external(.2)
        command = BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31)))
        first = controller.publish(command)
        sent_before_poll = list(transport.published)

        with patch.object(transport, 'poll_control_status',
                          side_effect=OSError('status receive failed')) as poll:
            with self.assertRaisesRegex(OSError, 'status receive failed'):
                controller.publish(command)
            self.assertEqual(poll.call_count, 1)

        self.assertEqual(transport.published, sent_before_poll)
        self.assertEqual(transport.close_count, 0)
        resumed = controller.publish(command)
        self.assertEqual((resumed.session_id, resumed.sequence), (42, 2))
        self.assertEqual(
            [message for topic, message in transport.published
             if topic == BODY_CMD_TOPIC], [first, resumed])

    def test_publish_and_status_callback_preserve_lock_order(self):
        c,t=self.fixture()
        c.enter_external(.2)
        pull=self.transport()
        callback_entered=threading.Event()
        publish_polling=threading.Event()
        errors=[]
        published=[]

        def callback(status):
            callback_entered.set()
            if not publish_polling.wait(1.0):
                raise AssertionError('publish did not reach status drain')
            if not c._condition.acquire(timeout=1.0):
                raise AssertionError('publish holds condition while waiting for status lock')
            try:
                c._on_status(status)
            finally:
                c._condition.release()

        pull._callbacks[CONTROL_STATUS_TOPIC]=[callback]
        pull._pulls[CONTROL_STATUS_TOPIC].samples.append(S(payload=ControlStatus(
            status_seq=2,current_action='EXTERNAL',phase=ExternalPhase.RUNNING,
            client_id=9,control_id=13,generation=84,descriptor_ref=DescriptorRef(11))))

        def poll():
            publish_polling.set()
            return pull.poll_control_status()

        t.poll_control_status=poll

        def drain():
            try:pull.poll_control_status()
            except BaseException as error:errors.append(error)

        def publish():
            try:published.append(c.publish(BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31)))))
            except BaseException as error:errors.append(error)

        receiver=threading.Thread(target=drain,daemon=True)
        sender=threading.Thread(target=publish,daemon=True)
        receiver.start()
        entered=callback_entered.wait(1.0)
        sender.start()
        receiver.join(3.0)
        sender.join(3.0)
        self.assertTrue(entered)
        self.assertFalse(receiver.is_alive() or sender.is_alive())
        self.assertEqual(errors,[])
        self.assertEqual([(m.session_id,m.sequence) for m in published],[(84,1)])

    def test_close_during_status_drain_rejects_publish_and_late_callback(self):
        c,t=self.fixture()
        initial=c.enter_external(.2)
        pull=self.transport()
        callback_entered=threading.Event()
        closed=threading.Event()
        errors=[]

        def callback(status):
            callback_entered.set()
            if not closed.wait(1.0):
                raise AssertionError('close blocked behind status drain')
            c._on_status(status)

        pull._callbacks[CONTROL_STATUS_TOPIC]=[callback]
        pull._pulls[CONTROL_STATUS_TOPIC].samples.append(S(payload=ControlStatus(
            status_seq=2,current_action='EXTERNAL',phase=ExternalPhase.RUNNING,
            client_id=9,control_id=13,generation=84,descriptor_ref=DescriptorRef(11))))
        t.poll_control_status=pull.poll_control_status

        def publish():
            try:c.publish(BodyCmd(joints=tuple(BodyJointCmd() for _ in range(31))))
            except BaseException as error:errors.append(error)

        def close():
            try:c.close()
            except BaseException as error:errors.append(error)
            finally:closed.set()

        sender=threading.Thread(target=publish,daemon=True)
        closer=threading.Thread(target=close,daemon=True)
        sender.start()
        entered=callback_entered.wait(1.0)
        closer.start()
        sender.join(3.0)
        closer.join(3.0)
        self.assertTrue(entered)
        self.assertFalse(sender.is_alive() or closer.is_alive())
        self.assertEqual(len(errors),1)
        self.assertIsInstance(errors[0],ControllerClosedError)
        self.assertIs(c.status,initial)
        self.assertFalse(any(topic==BODY_CMD_TOPIC for topic,_ in t.published))

    def test_entry_timeout_keeps_deadline_and_best_effort_cancel(self):
        c,t=self.fixture(True)
        with self.assertRaises(ExternalControlTimeoutError):c.enter_external(.03)
        self.assertEqual(t.published[-1][1].operation,ControlOperation.CANCEL)

    def test_rejection_uses_original_exception_and_cancel(self):
        from locomotion_aorta import ControlRejectedError
        c,t=self.fixture(reject=True)
        with self.assertRaises(ControlRejectedError):c.enter_external(.1)
        self.assertEqual(t.published[-1][1].operation,ControlOperation.CANCEL)

    def test_state_source_has_one_array_wait_api_and_close_is_readonly(self):
        calls=[];state=S(descriptor_ref=DescriptorRef(11));status=S(descriptor_ref=DescriptorRef(11))
        class Transport(FakeTransport):
            control_status=status
            def wait_state(self, timeout):
                calls.append(timeout);return state
            def close(self):
                calls.append('close');super().close()
        transport=Transport()
        source=PullStateSource(_transport=transport)
        source.get_robot_info(.1)
        self.assertIs(source.wait_state(.1),state);self.assertFalse(hasattr(source, "wait_body_state"))
        self.assertIs(source.control_status,status);source.close();self.assertEqual(calls,[.1,'close'])

class RuntimeIdentityPullR3Test(unittest.TestCase):
    def test_readonly_discovery_binds_before_accepting_body_without_control(self):
        from body_fixtures import robot_info_reply
        from locomotion_aorta.body_types import DescriptorRef
        class Reader(FakeTransport):
            def __init__(self):
                super().__init__()
                self.reply = replace(robot_info_reply(), descriptor_ref=DescriptorRef(11))
                self.state = S(descriptor_ref=DescriptorRef(11))
            def get_robot_info(self, timeout):
                return self.reply
            def wait_state(self, timeout):
                return self.state
        transport = Reader()
        source = PullStateSource(_transport=transport)
        self.addCleanup(source.close)
        with self.assertRaises(ValueError):
            source.wait_state(.1)
        self.assertEqual(source.get_robot_info(.1), transport.reply)
        self.assertIs(source.wait_state(.1), transport.state)
        transport.state = S(descriptor_ref=DescriptorRef(99))
        with self.assertRaises(ValueError):
            source.wait_state(.1)
        self.assertEqual(transport.published, [])
        transport.reply = replace(transport.reply, descriptor_ref=DescriptorRef(12))
        with self.assertRaisesRegex(ValueError, 'runtime changed'):
            source.get_robot_info(.1)
        transport.state = S(descriptor_ref=DescriptorRef(11))
        with self.assertRaisesRegex(ValueError, 'runtime changed'):
            source.wait_state(.1)

class PullCandidateCommitR3Test(unittest.TestCase):
    def make_source(self):
        from body_fixtures import robot_info_reply
        transport = PullTest().transport()
        transport._candidate_gate = None
        transport.get_robot_info = lambda timeout: robot_info_reply()
        transport.close = lambda: setattr(transport, '_closed', True)
        source = PullStateSource(_transport=transport)
        self.addCleanup(source.close)
        source.get_robot_info(.1)
        return source, transport

    def test_real_ring_rejects_foreign_runtime_before_both_state_caches(self):
        from test_body_codec import make_state
        source, transport = self.make_source()
        ring = transport._pulls[BODY_STATE_TOPIC]
        def sample(runtime, error, stamp):
            payload, _ = make_state(runtime_id=runtime, error_codes=(error,) * 31)
            return S(payload=payload, context=S(receive_stamp_ns=stamp))

        ring.samples.append(sample(11, 3, 41))
        arrays_a = source.wait_state(0)
        body_a = transport._wait_body_state(0)
        accepted_a = transport._latest_sample
        self.assertEqual((int(arrays_a.error_code[0]), body_a.joints[0].error_code), (3, 3))
        clocks = (arrays_a.received_monotonic_ns, arrays_a.received_realtime_ns,
                  arrays_a.pull_context_receive_stamp_ns)
        self.assertEqual(clocks[2], 41)

        ring.samples.append(sample(99, 7, 42))
        self.assertIs(source.wait_state(0), arrays_a)
        self.assertIs(transport._latest_sample, accepted_a)
        self.assertIs(transport._latest_body_state, body_a)
        self.assertIs(source.wait_state(0), arrays_a)
        self.assertEqual((arrays_a.received_monotonic_ns, arrays_a.received_realtime_ns,
                          arrays_a.pull_context_receive_stamp_ns), clocks)

        ring.samples.append(sample(99, 8, 43))
        self.assertIs(transport._wait_body_state(0), body_a)
        self.assertIs(transport._latest_sample, accepted_a)
        self.assertIs(transport._latest_state, arrays_a)
        self.assertIs(transport._wait_body_state(0), body_a)

        ring.samples.append(sample(11, 9, 44))
        body_c = transport._wait_body_state(0)
        self.assertEqual(body_c.joints[0].error_code, 9)
        arrays_c = source.wait_state(0)
        self.assertEqual(int(arrays_c.error_code[0]), 9)
        self.assertEqual(arrays_c.pull_context_receive_stamp_ns, 44)
        self.assertIsNot(transport._latest_sample, accepted_a)
        self.assertIsNot(arrays_c, arrays_a)

    def test_real_status_ring_keeps_last_valid_on_foreign_runtime(self):
        source, transport = self.make_source()
        ring = transport._pulls[CONTROL_STATUS_TOPIC]
        a = ControlStatus(1, 'PASSIVE', ExternalPhase.IDLE, descriptor_ref=DescriptorRef(11))
        b = ControlStatus(2, 'EXTERNAL', ExternalPhase.RUNNING, descriptor_ref=DescriptorRef(99))
        c = ControlStatus(3, 'PASSIVE', ExternalPhase.IDLE, descriptor_ref=DescriptorRef(11))
        ring.samples.append(S(payload=a))
        self.assertIs(source.control_status, a)
        ring.samples.append(S(payload=b))
        self.assertIs(source.control_status, a)
        self.assertIs(source.control_status, a)
        ring.samples.append(S(payload=replace(a, status_seq=2, protocol_version=2)))
        self.assertIs(source.control_status, a)
        ring.samples.append(S(payload=c))
        self.assertIs(source.control_status, c)

    def test_pull_controller_validates_client_before_its_transport_construction(self):
        created = []
        class Constructed(FakeTransport, PullTransport):
            def __init__(self, **kwargs):
                FakeTransport.__init__(self)
                self._candidate_gate = None
                created.append((kwargs, self))
        with patch('locomotion_aorta.pull.PullTransport', Constructed):
            for client_id in (True, 0, -1, '7', 1 << 64):
                with self.subTest(client_id=client_id), self.assertRaises(ValueError):
                    PullController(client_id=client_id)
                self.assertEqual(created, [])
            for client_id in (None, 7):
                controller = PullController(client_id=client_id)
                transport = created[-1][1]
                controller.close()
                controller.close()
                self.assertEqual(transport.close_count, 1)
            self.assertEqual(len(created), 2)

    def test_controller_uses_same_gate_for_native_and_array_ring_reads(self):
        from body_fixtures import robot_info_reply
        from test_body_codec import make_state
        transport = PullTest().transport()
        transport._candidate_gate = None
        transport.node = S(create_subscriber_typed=lambda *a, **kw: S(close=lambda: None))
        transport.subscriptions = []
        transport.get_robot_info = lambda timeout: robot_info_reply()
        transport.close = lambda: setattr(transport, '_closed', True)
        controller = PullController(client_id=7, _transport=transport)
        self.addCleanup(controller.close)
        controller.get_robot_info(.1)
        ring = transport._pulls[BODY_STATE_TOPIC]
        def sample(runtime, error, stamp):
            payload, _ = make_state(runtime_id=runtime, error_codes=(error,) * 31)
            return S(payload=payload, context=S(receive_stamp_ns=stamp))
        ring.samples.append(sample(11, 4, 61))
        body_a = controller.wait_body_state(0)
        arrays_a = controller.wait_state(0)
        accepted_a = transport._latest_sample
        ring.samples.append(sample(99, 8, 62))
        self.assertIs(controller.wait_body_state(0), body_a)
        self.assertIs(controller.wait_state(0), arrays_a)
        self.assertIs(transport._latest_sample, accepted_a)
        self.assertEqual(arrays_a.pull_context_receive_stamp_ns, 61)
        ring.samples.append(sample(11, 6, 63))
        arrays_c = controller.wait_state(0)
        self.assertEqual(int(arrays_c.error_code[0]), 6)
        self.assertEqual(controller.wait_body_state(0).joints[0].error_code, 6)
        self.assertEqual(arrays_c.pull_context_receive_stamp_ns, 63)
