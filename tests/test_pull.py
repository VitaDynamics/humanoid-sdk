"""Opt-in pull state/snapshot/session lifecycle; no network or motor IO."""
import threading
import unittest
import time
from collections import deque
from types import SimpleNamespace as S
from unittest.mock import patch
from locomotion_aorta import PullController,PullStateSource,PullTransport,ExternalControlTimeoutError
from locomotion_aorta import ControlAck,ControlStatus,ControlOperation,ExternalPhase
from locomotion_aorta import BodyCmd,BodyJointCmd,ControllerClosedError,NotRunningError
from locomotion_aorta.transport import BODY_CMD_TOPIC,BODY_STATE_TOPIC,CONTROL_STATUS_TOPIC,CONTROL_REQUEST_TOPIC
from fake_transport import FakeTransport

class Ring:
    def __init__(self):self.samples=deque();self.waits=[];self.closed=False
    def recv(self,timeout):self.waits.append(timeout);return self.samples.popleft() if self.samples else None
    def try_recv(self):return self.recv(0)
    def close(self):self.closed=True

class PullTest(unittest.TestCase):
    def transport(self):
        t=PullTransport.__new__(PullTransport);t._closed=False;t._state_lock=threading.Lock();t._status_lock=threading.RLock();t._status=None;t._latest_state=None;t._latest_sample=None;t._latest_body_state=None
        t._pulls={BODY_STATE_TOPIC:Ring(),CONTROL_STATUS_TOPIC:Ring()};t._callbacks={BODY_STATE_TOPIC:[],CONTROL_STATUS_TOPIC:[]}
        t._bindings=S(control_status_decoder=lambda v:v);t._subscriber_spec=lambda topic:(None,lambda v:v,None)
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

    def fixture(self,missing_status=False,reject=False):
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
            t.emit('/sdk/foot_humanoid/control/ack',ControlAck(request_id=req.request_id,operation=req.operation,accepted=not reject,lease_id='L'))
            if not missing_status and req.operation in(ControlOperation.COMMIT,ControlOperation.CANCEL):
                cancel=req.operation==ControlOperation.CANCEL
                t.emit(CONTROL_STATUS_TOPIC,ControlStatus(status_seq=2 if cancel else 1,current_action='PASSIVE'if cancel else'EXTERNAL',phase=ExternalPhase.IDLE if cancel else ExternalPhase.RUNNING,client_id='client',lease_id='L',generation=42))
        t.on_publish=publish;c=PullController(client_id='client',_transport=t);self.addCleanup(c.close);return c,t

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
            client_id='client',lease_id='L',generation=84))
        second=c.publish(command)
        self.assertEqual((first.session_id,first.sequence),(42,1))
        self.assertEqual((second.session_id,second.sequence),(84,1))
        t.emit(CONTROL_STATUS_TOPIC,ControlStatus(
            status_seq=3,current_action='PASSIVE',phase=ExternalPhase.IDLE,
            client_id='client',lease_id='L',generation=84))
        with self.assertRaises(NotRunningError):c.publish(command)
        self.assertEqual([m for topic,m in t.published if topic==BODY_CMD_TOPIC],[first,second])
        t.emit(CONTROL_STATUS_TOPIC,ControlStatus(
            status_seq=4,current_action='EXTERNAL',phase=ExternalPhase.RUNNING,
            client_id='client',lease_id='L',generation=84))
        resumed=c.publish(command)
        self.assertEqual((resumed.session_id,resumed.sequence),(84,2))

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
            client_id='client',lease_id='L',generation=84)))

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
            client_id='client',lease_id='L',generation=84)))
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
        calls=[];state=object();transport=S(wait_state=lambda timeout:(calls.append(timeout)or state),close=lambda:calls.append('close'),control_status='snapshot')
        source=PullStateSource(_transport=transport)
        self.assertIs(source.wait_state(.1),state);self.assertFalse(hasattr(source, "wait_body_state"))
        self.assertEqual(source.control_status,'snapshot');source.close();self.assertEqual(calls,[.1,'close'])
