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

    def test_readonly_stream_rejects_missing_runtime_before_node(self):
        import aorta
        from lowlevel.BodyState import BodyState as WireBodyState
        from locomotion_aorta.transport import AortaBindingsUnavailableError
        with mock.patch.object(WireBodyState, 'RuntimeId', None), mock.patch.object(aorta, 'Node') as node:
            with self.assertRaisesRegex(AortaBindingsUnavailableError, 'direct-runtime'):
                _StateStream(node_name='readonly-test', group='default', callback=lambda *_: None)
            node.assert_not_called()

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

    def test_info_rpc_cache_and_timeout_order_remain_per_source(self):
        from locomotion_aorta import transport as transport_module
        from locomotion_aorta import _state_stream as stream_module

        class Client:
            def __init__(self):
                self.matching = True
                self.waits = []
                self.headers = []
                self.closed = False
                self.close_error = None
            def close(self):
                if self.closed:
                    return
                self.closed = True
                if self.close_error is not None:
                    raise self.close_error
            def raw(self): return self
            def wait_for_matching(self, timeout_ms):
                self.waits.append(timeout_ms)
                return self.matching
            def call(self, fill):
                self.headers.append(fill('builder', 17))
                return SimpleNamespace(decode=lambda: 'actual-reply-view')

        def make_node():
            node = SimpleNamespace()
            node.clients = []
            node.children = []
            node.close = mock.Mock()
            def create(*args, **kwargs):
                self.assertFalse(any(not c.closed for c in node.clients))
                client = Client()
                node.clients.append(client)
                return client
            node.create_owned = mock.Mock(side_effect=create)
            def retaining_factory(*args, **kwargs):
                client = create(*args, **kwargs)
                node.children.append(client)
                return client
            node.create_client_typed = mock.Mock(side_effect=retaining_factory)
            return node

        typed_calls = []
        def typed(node, raw, decoder):
            typed_calls.append((node, raw, decoder))
            return raw
        aorta = SimpleNamespace(ClientOptions=lambda **kw: SimpleNamespace(**kw),
            Client=lambda node, *args, **kw: node.create_owned(*args, **kw), TypedClient=typed)
        request = SimpleNamespace(GetRobotInfoRequestStart=mock.Mock(),
                                  GetRobotInfoRequestAddAortaHeader=mock.Mock(),
                                  GetRobotInfoRequestEnd=mock.Mock(return_value=23))
        response = SimpleNamespace(GetRobotInfoResponse=SimpleNamespace(GetRootAs=object()))
        meta = SimpleNamespace(SCHEMA_HASH_HEX="fixed-request-schema")
        names = {'aorta.services.humanoid_locomotion.GetRobotInfoRequest': request,
                 'aorta.services.humanoid_locomotion.GetRobotInfoResponse': response,
                 'get_robot_info_schema_meta': meta}
        stream = _StateStream.__new__(_StateStream)
        stream.node, stream._aorta, stream._info_lock = make_node(), aorta, threading.Lock()
        stream._info_clients, stream._closed = {}, False
        transport = transport_module._AortaTransport.__new__(transport_module._AortaTransport)
        transport.node, transport._info_lock = make_node(), threading.Lock()
        transport._info_clients, transport._closed = {}, False
        transport._body_lock = threading.Lock()
        transport._bindings = SimpleNamespace(aorta=aorta, get_robot_info_request=request,
            get_robot_info_response=response, get_robot_info_schema_meta=meta)
        reply = robot_info_reply()
        with mock.patch.object(stream_module.importlib, 'import_module', side_effect=names.__getitem__), \
             mock.patch.object(transport_module, 'decode_robot_info_response', return_value=reply) as decode:
            for source in (stream, transport):
                self.assertIs(source.get_robot_info(.1001), reply)
                self.assertIs(source.get_robot_info(.1001), reply)
                first = source._info_clients[101]
                self.assertIs(source.get_robot_info(.2), reply)
                self.assertEqual(source.node.create_owned.call_count, 2)
                source.node.create_client_typed.assert_not_called()
                self.assertEqual(source.node.children, [])
                calls = source.node.create_owned.call_args_list
                self.assertEqual([call.kwargs['options'].timeout_ms for call in calls], [101, 200])
                self.assertTrue(all(call.args[0] == transport_module.GET_ROBOT_INFO_SERVICE and
                                    call.kwargs['options'].service_schema_hash_hex == meta.SCHEMA_HASH_HEX for call in calls))
                self.assertEqual(first.waits, [101, 101])
                self.assertEqual(first.headers, [23, 23])
                self.assertTrue(first.closed)
                self.assertEqual(list(source._info_clients), [200])
                source._info_clients[200].matching = False
                with self.assertRaisesRegex(TimeoutError, 'provider not found'):
                    source.get_robot_info(.2)
            self.assertEqual(decode.call_count, 6)
            self.assertIsNot(stream._info_clients[200], transport._info_clients[200])
            for source in (stream, transport):
                for ms in range(1000, 21000, 1000):
                    self.assertIs(source.get_robot_info(ms / 1000), reply)
                    self.assertEqual(list(source._info_clients), [ms])
                    self.assertEqual(sum(not c.closed for c in source.node.clients), 1)
                current = source._info_clients[20000]
                current.close_error = OSError('close failed')
                before = source.node.create_owned.call_count
                with self.assertRaisesRegex(OSError, 'close failed'):
                    source.get_robot_info(21.)
                self.assertEqual(source.node.create_owned.call_count, before)
                self.assertEqual(list(source._info_clients), [None])
                self.assertTrue(current.closed)  # Actual Client closes logically before free.
                current.close_error = None  # A second close would now be a misleading no-op.
                with self.assertRaisesRegex(RuntimeError, 'cleanup is unconfirmed'):
                    source.get_robot_info(21.)
                self.assertEqual(source.node.create_owned.call_count, before)
                self.assertIs(source._info_clients[None], current)
                for _ in range(2):
                    with self.assertRaisesRegex(RuntimeError, 'cleanup is unconfirmed'):
                        source.close()
                source.node.close.assert_called_once()
                self.assertIs(source._info_clients[None], current)
            self.assertTrue(all(c[2] is response.GetRobotInfoResponse.GetRootAs for c in typed_calls))
            stream._closed = transport._closed = True
            with self.assertRaises(ValueError): stream.get_robot_info('bad')
            with self.assertRaisesRegex(RuntimeError, 'closed'): transport.get_robot_info('bad')

    def test_info_uses_public_caller_owned_clients_without_node_retention(self):
        import gc
        import weakref
        import aorta
        from locomotion_aorta import transport as module
        lib = mock.Mock()
        lib.aorta_client_new_with_options.return_value = 0
        node = SimpleNamespace(_handle=object(), _name='owned-info-test',
                               create_client_typed=mock.Mock(), children=[])
        meta = SimpleNamespace(SCHEMA_HASH_HEX='fixed-request-schema')
        response = SimpleNamespace(GetRobotInfoResponse=SimpleNamespace(GetRootAs=object()))
        reply = robot_info_reply()
        clients = {}
        with mock.patch('aorta.client.loader.library', return_value=lib), \
             mock.patch('aorta.client.bindings.configure', return_value=lib), \
             mock.patch('aorta.client.build_client_options', return_value=(None, None)), \
             mock.patch('aorta.client.make_header_stamper', return_value=object()), \
             mock.patch.object(aorta.Client, 'wait_for_matching', return_value=True), \
             mock.patch.object(aorta.TypedClient, 'call', return_value=SimpleNamespace(decode=lambda: object())), \
             mock.patch.object(module, 'decode_robot_info_response', return_value=reply):
            self.assertIs(module._call_get_robot_info(node, aorta, clients, 100, object(), response, meta), reply)
            original = weakref.ref(clients[100].raw())
            self.assertIs(module._call_get_robot_info(node, aorta, clients, 200, object(), response, meta), reply)
            gc.collect()
            self.assertIsNone(original())
            node.create_client_typed.assert_not_called()
            self.assertEqual(node.children, [])
            self.assertEqual(lib.aorta_client_free.call_count, 1)
            module._close_info_client(clients)
            self.assertEqual(lib.aorta_client_free.call_count, 2)
            self.assertEqual(clients, {})

    def test_late_discovery_does_not_retire_new_readonly_binding(self):
        from dataclasses import replace
        source = BodyStateSource()
        self.addCleanup(source.close)
        entered, release, errors = threading.Event(), threading.Event(), []
        caller = threading.get_ident()
        def reply(_timeout):
            if threading.get_ident() != caller:
                entered.set()
                if not release.wait(2):
                    raise AssertionError('late-discovery barrier not released')
                return replace(robot_info_reply(), runtime_id=12)
            return robot_info_reply()
        source._stream.get_robot_info = reply
        def discover():
            try: source.get_robot_info(.1)
            except Exception as error: errors.append(error)
        worker = threading.Thread(target=discover)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertEqual(source.get_robot_info(.1).runtime_id, 11)
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], ValueError)
        self.assertIn('discovery context changed', str(errors[0]))
        source._stream.emit()
        self.assertEqual(source.snapshot(.1).body_state().runtime_id, 11)


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
            self.assertEqual(source.get_robot_info(.1).runtime_id, 11)
            source._stream.emit()
            original = source.snapshot(.1)
            source._stream.emit(make_state(runtime_id=99)[0])
            with self.assertRaises(TimeoutError):
                source.snapshot(.001, after=original)
            source._stream.emit()
            self.assertIsNot(source.snapshot(.1, after=original), original)
            from dataclasses import replace
            source._stream.get_robot_info = lambda timeout: replace(
                robot_info_reply(), runtime_id=12)
            with self.assertRaisesRegex(ValueError, 'new source'):
                source.get_robot_info(.1)
            self.assertEqual(source._runtime_binding.runtime_id, 11)
            self.assertTrue(source._runtime_binding.poisoned)
            with self.assertRaisesRegex(ValueError, 'new source'):
                source.snapshot(.01)
