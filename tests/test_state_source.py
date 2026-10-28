"""Actual public body31 payloads with an isolated state-only stream."""
import gc
import math
import struct
import threading
import unittest
import weakref
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest import mock

from fake_transport import FakeTransport
from body_fixtures import robot_info_reply
from test_body_codec import make_state, module
from locomotion_aorta import BodyStateSource, BodyStateSnapshot, ExternalController, BODY_STATE_TOPIC
from locomotion_aorta import state as state_module
from locomotion_aorta._state_stream import _StateStream


class FakeStream:
    def __init__(self, *, callback, **_kwargs):
        self.callback = callback
        self.close = mock.Mock()

    def emit(self, data=None):
        self.callback(make_state(runtime_id=11)[0] if data is None else data,
                      module('BodyState').BodyState.GetRootAs)

    def get_robot_info(self, timeout):
        return robot_info_reply()


class StateSourceTest(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(state_module, 'create_state_stream', side_effect=FakeStream)
        self.factory = patch.start()
        self.addCleanup(patch.stop)

    def source(self, **kwargs):
        source = BodyStateSource(**kwargs)
        self.addCleanup(source.close)
        source.get_robot_info(.1)
        return source

    def test_snapshot_caches_incompatible_body_revision_error(self):
        import flatbuffers
        from lowlevel import BodyState as module
        from locomotion_aorta.state import BodyStateSnapshot
        builder = flatbuffers.Builder(1)
        module.BodyStateStart(builder)
        module.BodyStateAddProtocolVersion(builder, 1)
        builder.Finish(module.BodyStateEnd(builder))
        payload = bytes(builder.Output())
        decoder = mock.Mock(wraps=module.BodyState.GetRootAs)
        snapshot = BodyStateSnapshot(payload, decoder, 1, 2)
        for _ in range(2):
            with self.assertRaisesRegex(ValueError, 'protocol_version'):
                snapshot.body_state()
        decoder.assert_called_once_with(payload)

    def test_default_timeout_and_after_use_received_object_identity(self):
        source = self.source()
        self.assertEqual(source.group, 'default')
        source._stream.emit()
        with mock.patch.object(state_module, '_deadline', wraps=state_module._deadline) as deadline:
            first = source.snapshot()
            deadline.assert_called_once_with(5.0)
        self.assertIs(first, source.snapshot())
        with self.assertRaises(TimeoutError) as error:
            source.snapshot(.001, after=first)
        self.assertIs(type(error.exception), TimeoutError)
        source._stream.emit()  # Same producer state_id, new receive object.
        second = source.snapshot(after=first)
        self.assertIsNot(first, second)
        self.assertEqual(first.body_state().state_id, second.body_state().state_id)

    def test_timeout_and_text_validation(self):
        source = self.source()
        for value in (0, -1, math.inf, -math.inf, math.nan, True, None, '0'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                source.snapshot(value)
        with self.assertRaises(TimeoutError):
            source.snapshot(.001)
        for kwargs in ({'group': ''}, {'group': None}, {'node_name': ''}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                BodyStateSource(**kwargs)

    def test_direct_clocks_and_readonly_properties(self):
        source = self.source()
        with mock.patch.object(state_module.time, 'monotonic_ns', return_value=7) as mono, \
             mock.patch.object(state_module.time, 'time_ns', return_value=900000000000001) as real:
            source._stream.emit()
        frame = source.snapshot()
        self.assertEqual(frame.received_monotonic_ns, 7)
        self.assertEqual(frame.received_realtime_ns, 900000000000001)
        mono.assert_called_once_with()
        real.assert_called_once_with()
        for name in ('received_monotonic_ns', 'received_realtime_ns'):
            with self.assertRaises(AttributeError):
                setattr(frame, name, 123)

    def test_immutable_payload_and_old_snapshot_survives_close(self):
        source = self.source()
        data = bytearray(make_state(runtime_id=11)[0])
        source._stream.emit(data)
        old = source.snapshot()
        data[:] = bytes(len(data))
        source._stream.emit()
        source.close()
        source.close()
        self.assertEqual(old.body_state().state_id, 91)
        self.assertTrue(math.isnan(old.body_state().joints[14].q))
        source._stream.close.assert_called_once_with()
        with self.assertRaisesRegex(RuntimeError, 'closed') as error:
            source.snapshot()
        self.assertIs(type(error.exception), RuntimeError)
        source._stream.emit()  # Late callback is harmless.
        self.assertEqual(old.body_state().state_id, 91)

    def test_lazy_decode_once_with_concurrent_readers_and_live_callback(self):
        source = self.source()
        entered, release = threading.Event(), threading.Event()
        original = state_module.decode_body_state
        def decode(view):
            entered.set()
            if not release.wait(2):
                raise RuntimeError('test decoder was not released')
            return original(view)
        with mock.patch.object(state_module, 'decode_body_state', side_effect=decode) as call:
            source._stream.emit()
            first = source.snapshot()
            call.assert_not_called()
            with ThreadPoolExecutor(max_workers=8) as pool:
                futures = [pool.submit(first.body_state) for _ in range(32)]
                try:
                    self.assertTrue(entered.wait(1))
                    # The receive callback never takes the decoder's cache lock.
                    source._stream.emit()
                    second = source.snapshot(after=first)
                    self.assertIsNot(first, second)
                finally:
                    release.set()
                values = [future.result(2) for future in futures]
            self.assertEqual(call.call_count, 1)
            self.assertTrue(all(value is values[0] for value in values))
            second.body_state()
            self.assertEqual(call.call_count, 2)

    def test_decode_failure_is_deferred_cached_and_never_replaced_with_old_data(self):
        source = self.source()
        source._stream.emit()
        good = source.snapshot()
        self.assertEqual(good.body_state().state_id, 91)
        # Structural checks pass, but a later non-structural decode can fail.
        source._stream.emit()
        bad = source.snapshot(after=good)
        failure = ValueError('deferred decode error')
        with mock.patch.object(state_module, 'decode_body_state', side_effect=failure) as decode:
            for _ in range(2):
                with self.assertRaisesRegex(ValueError, 'deferred decode error') as error:
                    bad.body_state()
                self.assertIs(error.exception, failure)
            decode.assert_called_once()
        self.assertEqual(good.body_state().state_id, 91)
        source._stream.emit()
        self.assertEqual(source.snapshot(after=bad).body_state().state_id, 91)

    def test_missing_or_wrong_joint_vector_never_replaces_lazy_snapshot(self):
        import flatbuffers
        source = self.source()
        source._stream.emit()
        good = source.snapshot()
        clocks = (good.received_monotonic_ns, good.received_realtime_ns)
        # The envelope and Ref are valid, but the Body joint vector is malformed.
        data = bytearray(make_state(runtime_id=11)[0])
        view = module('BodyState').BodyState.GetRootAs(data)
        vector = view._tab.Vector(view._tab.Offset(22))
        struct.pack_into('<I', data, vector - 4, 30)
        obj = module('BodyState').BodyStateT.InitFromObj(make_state(runtime_id=11)[1])
        obj.joints = None
        builder = flatbuffers.Builder(1)
        builder.Finish(obj.Pack(builder))
        for invalid in (data, bytes(builder.Output())):
            source._stream.emit(invalid)
            self.assertIs(source.snapshot(), good)
            self.assertEqual((good.received_monotonic_ns, good.received_realtime_ns), clocks)
        self.assertEqual(good.body_state().state_id, 91)
        source._stream.emit()
        self.assertEqual(source.snapshot(after=good).body_state().state_id, 91)

    def test_undecodable_envelope_does_not_replace_valid_snapshot(self):
        source = self.source()
        source._stream.emit()
        good = source.snapshot()
        source._stream.callback(b'bad', module('BodyState').BodyState.GetRootAs)
        with self.assertRaises(TimeoutError):
            source.snapshot(.001, after=good)
        self.assertEqual(good.body_state().state_id, 91)

    def test_close_wakes_waiter_using_the_default_deadline(self):
        source = self.source()
        entered = threading.Event()
        def wait():
            entered.set()
            return source.snapshot()
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(wait)
            self.assertTrue(entered.wait(1))
            source.close()
            with self.assertRaisesRegex(RuntimeError, 'closed'):
                future.result(1)

    def test_new_frame_wakes_after_snapshot_waiter(self):
        source = self.source()
        source._stream.emit()
        first = source.snapshot()
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(source.snapshot, 1, after=first)
            source._stream.emit()
            self.assertIsNot(future.result(1), first)

    def test_replaced_snapshot_releases_without_cyclic_gc(self):
        enabled = gc.isenabled()
        gc.disable()
        try:
            frame = BodyStateSnapshot(make_state()[0], module('BodyState').BodyState.GetRootAs, 1, 2)
            state = frame.body_state()
            reference = weakref.ref(frame)
            del frame
            self.assertIsNone(reference())
            self.assertEqual(state.state_id, 91)
        finally:
            if enabled:
                gc.enable()

    def test_default_controller_keeps_its_original_subscription(self):
        transport = FakeTransport()
        controller = ExternalController(_transport=transport)
        self.addCleanup(controller.close)
        self.assertIn(BODY_STATE_TOPIC, transport.callbacks)

    def test_state_only_adapter_needs_no_publisher_or_client_factory(self):
        node = SimpleNamespace(group=lambda:'test', create_subscriber=mock.Mock(return_value=object()), close=mock.Mock())
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

class RuntimeIdentityStateSourceR3Test(unittest.TestCase):
    def test_discovery_and_foreign_topic_do_not_create_publishers_or_rebind(self):
        with mock.patch.object(state_module, 'create_state_stream', side_effect=FakeStream):
            source = BodyStateSource()
            self.addCleanup(source.close)
            source._stream.emit()
            with self.assertRaisesRegex(ValueError, 'get_robot_info'):
                source.snapshot(.01)
            self.assertEqual(source.get_robot_info(.1).descriptor_ref.robot_runtime_id, 11)
            source._stream.emit()
            original = source.snapshot(.1)
            source._stream.emit(make_state(runtime_id=99)[0])
            with self.assertRaises(TimeoutError):
                source.snapshot(.001, after=original)
            source._stream.emit()
            self.assertIsNot(source.snapshot(.1, after=original), original)
