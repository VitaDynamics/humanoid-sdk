"""Real public generated payloads; fake stream isolates lifecycle from the network."""
import math
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest import mock

from fake_transport import FakeTransport
from test_body_codec import make_state, module
from locomotion_aorta import (
    BodyStateSource, FrameToken, StateSourceClosedError, StateSourceTimeoutError,
    ExternalController, ControllerClosedError, ExternalControlTimeoutError,
    BODY_STATE_TOPIC, CONTROL_ACK_TOPIC, CONTROL_STATUS_TOPIC, ControlStatus,
)
from locomotion_aorta import state as state_module
from locomotion_aorta._state_stream import _StateStream


class FakeStream:
    def __init__(self, *, callback, **_kwargs):
        self.callback = callback
        self.close = mock.Mock()

    def emit(self, data=None):
        if data is None:
            data = make_state()[0]
        self.callback(data, module('BodyState').BodyState.GetRootAs)


class StateSourceTest(unittest.TestCase):
    def setUp(self):
        self.patch = mock.patch.object(state_module, 'create_state_stream', side_effect=FakeStream)
        self.factory = self.patch.start()
        self.addCleanup(self.patch.stop)

    def source(self, conversion='on_demand', group='test'):
        source = BodyStateSource(group=group, conversion=conversion)
        self.addCleanup(source.close)
        return source

    def test_zero_timeout_poll_and_after_token(self):
        s = self.source()
        with self.assertRaises(StateSourceTimeoutError):
            s.snapshot(0)
        s._stream.emit()
        a = s.snapshot(0)
        self.assertIs(a, s.snapshot(0))
        with self.assertRaises(StateSourceTimeoutError):
            s.snapshot(0, after_token=a.token)
        s._stream.emit()  # Same state_id is a distinct local receive event.
        b = s.snapshot(0, after_token=a.token)
        self.assertEqual(a.metadata.state_id, b.metadata.state_id)
        self.assertEqual(b.token.sequence, a.token.sequence + 1)
        self.assertNotEqual(a.token, b.token)

    def test_argument_validation_and_source_identity(self):
        s, other = self.source(), self.source()
        other._stream.emit()
        for value in (-1, -math.inf, math.nan, True, None, '0'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                s.snapshot(value)
        for token in (other.snapshot(0).token, 'bad', FrameToken(s._source_id, 0),
                      FrameToken(s._source_id, True)):
            with self.subTest(token=token), self.assertRaises(ValueError):
                s.snapshot(0, after_token=token)
        for args in ({'group': '', 'conversion': 'eager'},
                     {'group': 'a', 'conversion': 'automatic'},
                     {'group': 'a', 'conversion': 'eager', 'node_name': ''}):
            with self.assertRaises(ValueError):
                BodyStateSource(**args)

    def test_direct_realtime_and_monotonic_reads_and_readonly_properties(self):
        s = self.source()
        with mock.patch.object(state_module.time, 'monotonic_ns', return_value=7) as mono, \
             mock.patch.object(state_module.time, 'time_ns', return_value=900000000000001) as real:
            s._stream.emit()
        snapshot = s.snapshot(0)
        self.assertEqual(snapshot.received_monotonic_ns, 7)
        self.assertEqual(snapshot.received_realtime_ns, 900000000000001)
        mono.assert_called_once_with()
        real.assert_called_once_with()
        for name in ('token', 'metadata', 'received_monotonic_ns', 'received_realtime_ns'):
            with self.assertRaises(AttributeError):
                setattr(snapshot, name, 123)

    def test_immutable_payload_and_old_snapshot_survives_close(self):
        s = self.source()
        data = bytearray(make_state()[0])
        s._stream.emit(data)
        old = s.snapshot(0)
        data[:] = bytes(len(data))
        s._stream.emit()
        s.close()
        s.close()
        self.assertEqual(old.body_state().state_id, 91)
        self.assertTrue(math.isnan(old.body_state().joints[14].q))
        s._stream.close.assert_called_once_with()
        with self.assertRaises(StateSourceClosedError):
            s.snapshot(0)
        s._stream.emit()  # Late callback is ignored.
        self.assertEqual(old.token.sequence, 1)

    def test_eager_and_on_demand_conversion_and_multi_reader_cache(self):
        original = state_module.decode_body_state
        for conversion in ('eager', 'on_demand'):
            with self.subTest(conversion=conversion):
                s = self.source(conversion)
                with mock.patch.object(state_module, 'decode_body_state', wraps=original) as decode:
                    s._stream.emit()
                    snapshot = s.snapshot(0)
                    self.assertEqual(decode.call_count, int(conversion == 'eager'))
                    with ThreadPoolExecutor(max_workers=8) as pool:
                        values = list(pool.map(lambda _: snapshot.body_state(), range(32)))
                    self.assertEqual(decode.call_count, 1)
                    self.assertTrue(all(v is values[0] for v in values))
                    s._stream.emit()
                    s.snapshot(0).body_state()
                    self.assertEqual(decode.call_count, 2)

    def test_decode_failure_is_not_cached_or_replaced_with_previous_frame(self):
        s = self.source()
        s._stream.emit()
        good = s.snapshot(0)
        s._stream.emit(b'bad')
        bad = s.snapshot(0, after_token=good.token)
        for _ in range(2):
            with self.assertRaises(Exception) as error:
                bad.body_state()
            self.assertNotIsInstance(error.exception, StateSourceTimeoutError)
        self.assertEqual(good.body_state().state_id, 91)

    def test_eager_callback_error_matches_full_decoder_and_can_recover(self):
        s = self.source('eager')
        with self.assertRaises(Exception) as eager:
            s._stream.emit(b'bad')
        with self.assertRaises(type(eager.exception)):
            state_module.decode_body_state(module('BodyState').BodyState.GetRootAs(b'bad'))
        with self.assertRaises(StateSourceTimeoutError):
            s.snapshot(0)
        s._stream.emit()
        self.assertEqual(s.snapshot(0).metadata.state_id, 91)

    def test_source_close_wakes_unbounded_waiter(self):
        s = self.source()
        started = threading.Event()
        def wait():
            started.set()
            return s.snapshot(math.inf)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(wait)
            self.assertTrue(started.wait(1))
            s.close()
            with self.assertRaises(StateSourceClosedError):
                future.result(1)

    def test_new_frame_wakes_after_token_waiter(self):
        s = self.source()
        s._stream.emit()
        first = s.snapshot(0)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(s.snapshot, 1, after_token=first.token)
            s._stream.emit()
            self.assertGreater(future.result(1).token.sequence, first.token.sequence)

    def test_concurrent_eager_callbacks_do_not_publish_older_frame_last(self):
        s = self.source('eager')
        entered, release = threading.Event(), threading.Event()
        original = state_module.decode_body_state
        def decode(view):
            if threading.current_thread().name == 'older-frame':
                entered.set()
                self.assertTrue(release.wait(2))
            return original(view)
        with mock.patch.object(state_module, 'decode_body_state', side_effect=decode):
            thread = threading.Thread(target=s._stream.emit, name='older-frame')
            thread.start()
            self.assertTrue(entered.wait(1))
            s._stream.emit()
            self.assertEqual(s.snapshot(0).token.sequence, 2)
            release.set()
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(s.snapshot(0).token.sequence, 2)

    def test_injection_omits_second_state_subscription_and_preserves_control_topics(self):
        s = self.source()
        s._stream.emit()
        transport = FakeTransport()
        c = ExternalController(group='test', state_source=s, _transport=transport)
        self.addCleanup(c.close)
        self.assertEqual(set(transport.callbacks), {CONTROL_ACK_TOPIC, CONTROL_STATUS_TOPIC})
        self.assertEqual(c.wait_body_state(.1).state_id, 91)
        self.assertEqual(transport.published, [])
        c.close()
        self.assertEqual(s.snapshot(0).body_state().state_id, 91)
        s._stream.close.assert_not_called()

    def test_bad_source_rejected_before_control_transport_creation(self):
        s = self.source()
        with mock.patch('locomotion_aorta.controller.create_aorta_transport') as create:
            for source, group, error in ((s, 'other', ValueError), (object(), 'test', ValueError)):
                with self.assertRaises(error):
                    ExternalController(group=group, state_source=source)
            s.close()
            with self.assertRaises(StateSourceClosedError):
                ExternalController(group='test', state_source=s)
            create.assert_not_called()

    def test_borrowed_source_close_and_controller_close_wake_own_waiters(self):
        for close_source in (False, True):
            with self.subTest(close_source=close_source):
                s = self.source()
                c = ExternalController(group='test', state_source=s, _transport=FakeTransport())
                self.addCleanup(c.close)
                with ThreadPoolExecutor(max_workers=1) as pool:
                    future = pool.submit(c.wait_body_state, 10)
                    (s if close_source else c).close()
                    with self.assertRaises(StateSourceClosedError if close_source else ControllerClosedError):
                        future.result(1)

    def test_injected_timeout_remains_external_control_timeout(self):
        s = self.source()
        c = ExternalController(group='test', state_source=s, _transport=FakeTransport())
        self.addCleanup(c.close)
        with self.assertRaises(ExternalControlTimeoutError):
            c.wait_body_state(.001)

    def test_slow_decode_does_not_block_status_or_close(self):
        s = self.source()
        s._stream.emit()
        transport = FakeTransport()
        c = ExternalController(group='test', state_source=s, _transport=transport)
        self.addCleanup(c.close)
        entered, release = threading.Event(), threading.Event()
        original = state_module.decode_body_state
        def decode(view):
            entered.set()
            self.assertTrue(release.wait(2))
            return original(view)
        with mock.patch.object(state_module, 'decode_body_state', side_effect=decode), \
             ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(c.wait_body_state, 1)
            self.assertTrue(entered.wait(1))
            status = ControlStatus(status_seq=1, current_action='PASSIVE', phase='IDLE')
            transport.emit(CONTROL_STATUS_TOPIC, status)
            self.assertIs(c.status, status)
            c.close()
            release.set()
            with self.assertRaises(ControllerClosedError):
                future.result(1)

    def test_cached_borrowed_source_close_does_not_return_old_dto(self):
        s = self.source()
        s._stream.emit()
        c = ExternalController(group='test', state_source=s, _transport=FakeTransport())
        self.addCleanup(c.close)
        c.wait_body_state(.1)
        s.close()
        with self.assertRaises(StateSourceClosedError):
            c.wait_body_state(.1)

    def test_injected_session_keeps_ack_sequence_population_and_cancel_contract(self):
        from test_controller import ControllerHarness, make_command
        from locomotion_aorta import BODY_CMD_TOPIC, CONTROL_REQUEST_TOPIC, ControlOperation, ExternalPhase
        harness = ControllerHarness()
        harness.controller.close()
        harness.transport = FakeTransport()
        s = self.source(group='default')
        s._stream.emit()
        c = ExternalController(client_id='test-client', state_source=s, _transport=harness.transport)
        harness.controller = c
        self.addCleanup(c.close)
        harness.accept_enter()
        status = c.enter_external(1)
        self.assertEqual(status.phase, ExternalPhase.RUNNING)
        # No wait_body_state call: publish still learns populated flags from borrowed source.
        from dataclasses import replace
        command = make_command(17)
        joints = list(command.joints)
        joints[14] = replace(joints[14], q=math.nan)
        c.publish(replace(command, joints=tuple(joints)))
        c.publish(command)
        commands = [m for topic, m in harness.transport.published if topic == BODY_CMD_TOPIC]
        self.assertEqual([m.sequence for m in commands], [1, 2])
        self.assertTrue(all(m.session_id == status.generation for m in commands))
        c.close()
        cancels = [m for topic, m in harness.transport.published
                   if topic == CONTROL_REQUEST_TOPIC and m.operation == ControlOperation.CANCEL]
        self.assertEqual(len(cancels), 1)
        self.assertEqual(s.snapshot(0).metadata.state_id, 91)

    def test_default_controller_still_uses_original_body_subscription(self):
        transport = FakeTransport()
        c = ExternalController(_transport=transport)
        self.addCleanup(c.close)
        self.assertIn(BODY_STATE_TOPIC, transport.callbacks)

    def test_state_only_adapter_has_no_publisher_or_client_factory(self):
        # Deliberately no publisher/client methods: calling either fails the test.
        node = SimpleNamespace(create_subscriber=mock.Mock(return_value=object()), close=mock.Mock())
        aorta = SimpleNamespace(Node=mock.Mock(return_value=node),
                                QoS=SimpleNamespace(sensor_data=lambda: 'sensor'))
        decoder = module('BodyState')
        with mock.patch('locomotion_aorta._state_stream.importlib.import_module',
                        side_effect=lambda name: aorta if name == 'aorta' else decoder):
            callback = mock.Mock()
            stream = _StateStream(node_name='read-only', group='test', callback=callback)
            args, kwargs = node.create_subscriber.call_args
            self.assertEqual(args[0], BODY_STATE_TOPIC)
            self.assertEqual(kwargs, {'qos': 'sensor'})
            args[1](b'payload', object())
            callback.assert_called_once_with(b'payload', decoder.BodyState.GetRootAs)
            stream.close()
            node.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
