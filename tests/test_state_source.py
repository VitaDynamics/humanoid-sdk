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
    ExternalController,
    BODY_STATE_TOPIC,
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
        with self.assertRaises(type(eager.exception)):
            s.snapshot(0).body_state()
        s._stream.emit()
        good=s.snapshot(0)
        self.assertEqual(good.metadata.state_id, 91)
        with self.assertRaises(type(eager.exception)):s._stream.emit(b'bad')
        received=s.snapshot(0,after_token=good.token)
        with self.assertRaises(type(eager.exception)):received.body_state()
        self.assertEqual(good.body_state().state_id,91)

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

    def test_large_finite_wait_does_not_overflow_platform_wait_and_close_wakes_it(self):
        s=self.source()
        entered=threading.Event();original=s._condition.wait
        def wait(timeout):
            self.assertLessEqual(timeout,threading.TIMEOUT_MAX)
            entered.set()
            return original(timeout)
        with mock.patch.object(s._condition,'wait',side_effect=wait), ThreadPoolExecutor(max_workers=1) as pool:
            future=pool.submit(s.snapshot,1e300)
            self.assertTrue(entered.wait(1));s.close()
            with self.assertRaises(StateSourceClosedError):future.result(1)

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









    def test_public_source_has_only_whole_frame_access(self):
        import inspect
        import locomotion_aorta as sdk
        s=self.source();s._stream.emit();frame=s.snapshot(0)
        self.assertFalse(hasattr(frame,'select'))
        self.assertFalse(hasattr(frame,'applied_cmd'))
        self.assertFalse(hasattr(sdk,'JointSelection'))
        self.assertFalse(hasattr(sdk,'JointColumns'))
        self.assertNotIn('state_source',inspect.signature(ExternalController).parameters)

    def test_replaced_snapshot_releases_without_cyclic_gc(self):
        import gc,weakref
        class TrackedSnapshot(state_module.BodyStateSnapshot):
            __slots__=('__weakref__',)
        enabled=gc.isenabled();gc.disable()
        try:
            frame=TrackedSnapshot(make_state()[0],module('BodyState').BodyState.GetRootAs,
                                  FrameToken('lifetime',1),1,2)
            state=frame.body_state();metadata=frame.metadata
            reference=weakref.ref(frame)
            del frame
            self.assertIsNone(reference())
            self.assertEqual(state.state_id,metadata.state_id)
        finally:
            if enabled:gc.enable()

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
