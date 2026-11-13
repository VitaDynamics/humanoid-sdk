"""Byte-exact Python oracle and pinned raw transport; no nodes or robot IO."""
import ctypes
import dataclasses
import importlib
import math
import os
import random
import struct
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest import mock

import flatbuffers
from locomotion_aorta import BodyCmd, BodyJointCmd, InvalidCommandError
from locomotion_aorta.controller import _validate_command
from locomotion_aorta.transport import _fill_body_cmd, _AortaTransport, BODY_CMD_TOPIC

from test_body_codec import make_command
from test_controller import ControllerHarness

try:
    from locomotion_aorta import _body_cmd_native as native
except ImportError:
    native = None


def oracle(command, stamp=123, node='codec', sequence=0):
    from aorta.header import build_aorta_header
    builder = flatbuffers.Builder(256)
    header = build_aorta_header(builder, publish_stamp_ns=stamp,
                                publisher_node=node, sequence=sequence)
    root = _fill_body_cmd(SimpleNamespace(body_cmd_module=importlib.import_module('lowlevel.BodyCmd')),
                          command, builder, header)
    builder.Finish(root)
    return bytes(builder.Output())


_BODY_TEST_TARGETS = tuple('fake_joint_' + str(i) for i in range(31))

class Publisher:
    def __init__(self, *, include_node=True, node_name="test-node"):
        self._seq_lock = threading.Lock()
        self._sequence = 0
        self._closed = False
        self._include_node = include_node
        self._node_name = node_name
        self._lib = SimpleNamespace(aorta_now_ns=self.clock)
        self.stamp, self.sent = 1234567, []
        self.failure = None

    def native_header_binding(self):
        # Synthetic public binding over real native HeaderContext. Actual bee
        # Core exports/ABI are a separate required Pkg lane, not proven here.
        from aorta._ffi.structs import AortaNativeHeader
        reserve_type = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p,
                                       ctypes.POINTER(AortaNativeHeader))
        free_type = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p)
        self.contexts = set()
        self.freed = []
        self.reserve_error = 0
        self.clock_error = 0
        self.free_error = 0
        def reserve(context, output):
            if context not in self.contexts or self._closed:
                return -2
            if self.reserve_error:
                return self.reserve_error
            with self._seq_lock:
                seq = self._sequence
                self._sequence += 1
            if self.clock_error:
                return self.clock_error
            output.contents.sequence = seq
            output.contents.publish_stamp_ns = self.stamp
            return 0
        def free(context):
            if context not in self.contexts:
                return -1
            self.contexts.remove(context)
            self.freed.append(context)
            return self.free_error
        self._callbacks = reserve_type(reserve), free_type(free)
        def acquire():
            context = len(self.freed) + len(self.contexts) + 1
            self.contexts.add(context)
            return context
        return SimpleNamespace(
            publisher_node=self._node_name.encode('utf-8') if self._include_node else b'',
            include_publisher_node=self._include_node,
            reserve_address=ctypes.cast(self._callbacks[0], ctypes.c_void_p).value,
            free_address=ctypes.cast(self._callbacks[1], ctypes.c_void_p).value,
            acquire_context=acquire, close=mock.Mock(), library_keepalive=self)

    def clock(self, ptr):
        ctypes.cast(ptr, ctypes.POINTER(ctypes.c_int64))[0] = self.stamp
        return 0

    def publish_bytes(self, data):
        if self.failure:
            raise self.failure
        self.sent.append(data)


@unittest.skipIf(native is None, 'installed native codec required; pkg must run this suite without skips')
class NativeBodyCmdTest(unittest.TestCase):
    def test_motor_theta_v4_preserves_all_fields_in_native_and_python_encoders(self):
        from locomotion_aorta import BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION
        from lowlevel.BodyCmd import BodyCmd as WireBodyCmd
        self.assertEqual(BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION, 4)
        self.assertEqual(native.BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION, 4)
        for version in (3, 4):
            command = dataclasses.replace(make_command(), protocol_version=version)
            wire = WireBodyCmd.GetRootAs(self.assertBytes(command))
            self.assertEqual(wire.ProtocolVersion(), version)
            self.assertEqual((wire.RuntimeId(), wire.ControlId(), wire.SessionId(),
                              wire.Sequence(), wire.InputStateId()), (11, 13, 42, 7, 123))
            for i, joint in enumerate(command.joints):
                decoded = wire.Joints(i)
                self.assertEqual((decoded.Q(), decoded.Dq(), decoded.Tau(), decoded.Kp(), decoded.Kd()),
                                 (joint.q, joint.dq, joint.tau, joint.kp, joint.kd))

    def test_motor_theta_v4_is_external_only(self):
        from locomotion_aorta import BodyCmdAction
        for action in (BodyCmdAction.HUMANOID_TRACKING, BodyCmdAction.UNKNOWN):
            command = dataclasses.replace(make_command(), protocol_version=4, action=action)
            with self.subTest(action=action):
                with self.assertRaises(ValueError):
                    native.encode(command, 0, None, 0)
                with self.assertRaises(ValueError):
                    oracle(command)

    def assertBytes(self, command, stamp=123, node='codec', sequence=0):
        expected = oracle(command, stamp, node, sequence)
        actual, trace = native.encode(command, stamp, node, sequence, True)
        self.assertEqual(actual, expected)
        self.assertGreaterEqual(trace['encode_native_ns'], 0)
        return actual

    def test_seeded_random_headers_and_vectors_are_byte_exact(self):
        rng = random.Random(136)
        for i in range(256):
            joints = tuple(BodyJointCmd(*(rng.uniform(-20, 20) for _ in range(5))) for _ in range(31))
            command = BodyCmd(joints, session_id=rng.randrange(2**64), sequence=rng.randrange(2**64),
                              input_state_id=rng.randrange(2**64), input_state_ts_real_us=i,
                              ts_client_created_us=i, ts_client_send_us=i+1,
                              runtime_id=2**64-1, control_id=2**64-2)
            self.assertBytes(command, rng.randrange(2**64), ['codec', '', None, '机器人\x00SDK'][i % 4],
                             [0, 1, 2**64-1][i % 3])

    def test_defaults_root_quat_signed_zero_subnormal_and_action_presence(self):
        for value in [0., -0., 2**-149, -2**-149, float('nan'), float('inf'), -float('inf')]:
            command = dataclasses.replace(make_command(), joints=(BodyJointCmd(q=value),)*31,
                                          root_quat=(1., -0., 0., .5))
            self.assertBytes(command, 0, None, 0)
        data = self.assertBytes(BodyCmd((BodyJointCmd(),)*31, runtime_id=11, control_id=13), 0, '', 0)
        view = importlib.import_module('lowlevel.BodyCmd').BodyCmd.GetRootAs(data)
        self.assertNotEqual(view._tab.Offset(6), 0)  # action id1 even at default18

    def test_controller_rejects_invalid_metadata_before_native_encoding(self):
        for field in ('input_state_id', 'input_state_ts_real_us',
                      'ts_client_created_us'):
            invalid_values = [-1, 2**64, 1.5, 'x']
            if field != 'ts_client_created_us':
                invalid_values.append(None)
            for value in invalid_values:
                with self.subTest(field=field, value=value), self.assertRaises(InvalidCommandError):
                    _validate_command(dataclasses.replace(make_command(), **{field: value}))

    def test_controller_fills_unset_creation_times_before_encoding(self):
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        h.accept_enter()
        h.controller.enter_external(.1)
        for created in (None, 0.0, 0):
            with self.subTest(created=created), mock.patch(
                    'locomotion_aorta.controller.time.monotonic_ns', return_value=1234567890):
                sent = h.controller.publish(dataclasses.replace(
                    make_command(), ts_client_created_us=created), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
            self.assertEqual(sent.ts_client_created_us, 1234567)
            self.assertIsInstance(sent.ts_client_created_us, int)
            self.assertBytes(sent)

    def test_controller_keeps_numpy_metadata_and_overrides_ignored_fields(self):
        import numpy as np
        h = ControllerHarness()
        self.addCleanup(h.controller.close)
        h.accept_enter()
        h.controller.enter_external(.1)
        for integer in (np.int64, np.uint64):
            command = dataclasses.replace(make_command(), input_state_id=integer(5),
                input_state_ts_real_us=integer(6), ts_client_created_us=integer(7),
                ts_client_send_us='ignored', root_quat=object())
            sent = h.controller.publish(command, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
            self.assertEqual((sent.input_state_id, sent.input_state_ts_real_us,
                              sent.ts_client_created_us), (5, 6, 7))
            self.assertEqual(sent.root_quat, (1., 0., 0., 0.))
            self.assertIsInstance(sent.ts_client_send_us, int)
            self.assertBytes(sent)

    def test_native_schema_constants_match_vendor_provenance(self):
        import json
        from pathlib import Path
        provenance = json.loads((Path(__file__).resolve().parents[1] /
            'demo_onnx/cpp/body_cmd_vendor/provenance.json').read_text())
        self.assertEqual(native.BODY_SCHEMA_SHA256, provenance['body_cmd_bfbs_sha256'])
        self.assertEqual(native.BODY_STATE_SCHEMA_SHA256, provenance['body_state_bfbs_sha256'])
        import hashlib
        root = Path(__file__).resolve().parents[1]
        compatibility = json.loads((root / 'compatibility.json').read_text())
        self.assertEqual(provenance['generated_source_wheel_version'], compatibility['humanoid_msgs_version'])
        wheel = 'vbot_humanoid_msgs-' + compatibility['humanoid_msgs_version'] + '-py3-none-any.whl'
        self.assertEqual(provenance['generated_source_wheel_sha256'], compatibility['wheel_sha256'][wheel])
        vendor = root / 'demo_onnx/cpp/body_cmd_vendor'
        for stem in ('body_cmd', 'body_state'):
            header = vendor / 'message/schemas/topic/lowlevel' / (stem + '_generated.h')
            self.assertEqual(hashlib.sha256(header.read_bytes()).hexdigest(), provenance[stem + '_generated_h_sha256'])
            self.assertEqual(provenance[stem + '_bfbs_sha256'], compatibility['schema_sha256'][stem])
        self.assertEqual(set(provenance['generated_include_sha256']), {
            'message/schemas/topic/lowlevel/body_common_generated.h',
            'message/schemas/topic/lowlevel/ee_common_generated.h',
            'message/schemas/system/aorta_base_generated.h',
        })
        for path, expected in provenance['generated_include_sha256'].items():
            with self.subTest(generated_include=path):
                self.assertEqual(hashlib.sha256((vendor / path).read_bytes()).hexdigest(), expected)
        for path, expected in provenance['generated_headers_sha256'].items():
            with self.subTest(native_archive_header=path):
                self.assertEqual(hashlib.sha256((vendor / path).read_bytes()).hexdigest(), expected)
        for path, expected in provenance['flatbuffers_include_sha256'].items():
            with self.subTest(flatbuffers_include=path):
                self.assertEqual(hashlib.sha256((vendor / path).read_bytes()).hexdigest(), expected)

    def test_controller_validation_population_and_type_rules_retained(self):
        for value in [True, 'x', math.nan, math.inf]:
            joints = list(make_command().joints);joints[0] = BodyJointCmd(q=value)
            with self.assertRaises(InvalidCommandError): _validate_command(BodyCmd(tuple(joints)))
        for value in [True, math.nan, math.inf]:
            joints = list(make_command().joints);joints[14] = BodyJointCmd(q=value);c = BodyCmd(tuple(joints))
            with self.assertRaises(InvalidCommandError): _validate_command(c)
            self.assertBytes(dataclasses.replace(c, runtime_id=11, control_id=13))
        _validate_command(make_command())
        for count in [0, 30, 32]:
            with self.assertRaises(InvalidCommandError): _validate_command(BodyCmd((BodyJointCmd(),)*count))
            with self.assertRaises(ValueError): native.encode(BodyCmd((BodyJointCmd(),)*count), 0, None, 0)

    def test_raw_adapter_keeps_core_clock_node_policy_and_failure_sequence(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        for node in [True, False]:
            pub = Publisher(include_node=node, node_name='test-μ-node')
            adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
            seq = pub._sequence
            adapter.publish(make_command(), guard=lambda send: send())
            self.assertEqual(pub.sent[-1], oracle(make_command(), pub.stamp,
                                                 pub._node_name if node else None, seq))
            adapter.close()
            self.assertEqual(pub.freed, [1])
        pub = Publisher();adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        pub.failure = OSError('send failed');seq = pub._sequence
        with self.assertRaisesRegex(OSError, 'send failed'): adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub._sequence, seq+1)
        pub.failure = None;pub._closed = True
        with self.assertRaises(Exception): adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub._sequence, seq+1)

    def test_nested_publish_cannot_overwrite_outer_joint_values(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        outer = dataclasses.replace(make_command(), joints=(BodyJointCmd(q=0.25),) * 31)
        inner = dataclasses.replace(make_command(), joints=(BodyJointCmd(q=-0.5),) * 31)

        class ReentrantCommand:
            def __getattr__(self, name):
                return getattr(outer, name)

            @property
            def protocol_version(self):
                adapter.publish(inner, guard=lambda send: send())
                return outer.protocol_version

        adapter.publish(ReentrantCommand(), guard=lambda send: send())
        self.assertEqual(pub._sequence, 2)
        self.assertEqual(pub.sent, [oracle(inner, pub.stamp, pub._node_name, 1),
                                    oracle(outer, pub.stamp, pub._node_name, 0)])

    def test_native_header_rejection_clock_error_and_context_close(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        from aorta.errors import BusyError, InternalError, NotFoundError
        pub = Publisher()
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        pub.reserve_error = -4
        with self.assertRaises(BusyError):
            adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub._sequence, 0)
        self.assertFalse(pub.sent)
        pub.reserve_error = 0
        pub.clock_error = -5
        with self.assertRaises(InternalError):
            adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub._sequence, 1)
        self.assertFalse(pub.sent)
        pub.clock_error = 0
        adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub._sequence, 2)
        adapter.close(); adapter.close()
        self.assertEqual(pub.freed, [1])
        with self.assertRaises(NotFoundError):
            adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub._sequence, 2)

    def test_header_free_internal_consumes_handle_without_second_free(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        from aorta.errors import InternalError
        pub = Publisher()
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        pub.free_error = -5
        with self.assertRaises(InternalError):
            adapter.close()
        self.assertEqual(pub.freed, [1])
        adapter.close()
        self.assertEqual(pub.freed, [1])

    def test_header_failure_precedes_command_getters_and_final_send(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        from aorta.errors import BusyError, InternalError
        pub = Publisher()
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        self.addCleanup(adapter.close)
        inspected = []
        gate = mock.Mock(side_effect=lambda send: send())

        class Command:
            @property
            def joints(self):
                inspected.append('joints')
                raise AssertionError('command evaluated after header failure')

        pub.reserve_error = -4
        with self.assertRaises(BusyError):
            adapter.publish(Command(), guard=gate)
        self.assertEqual(pub._sequence, 0)
        pub.reserve_error, pub.clock_error = 0, -5
        with self.assertRaises(InternalError):
            adapter.publish(Command(), guard=gate)
        self.assertEqual(pub._sequence, 1)
        pub.clock_error, pub.stamp = 0, -1
        with self.assertRaisesRegex(TypeError, 'timestamp'):
            adapter.publish(Command(), guard=gate)
        self.assertEqual(pub._sequence, 2)
        self.assertEqual(inspected, [])
        gate.assert_not_called()
        self.assertEqual(pub.sent, [])

    def test_public_header_encoding_still_requires_final_connection_gate(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        self.addCleanup(adapter.close)
        denied = mock.Mock(side_effect=InvalidCommandError('ownership changed'))
        with self.assertRaisesRegex(InvalidCommandError, 'ownership changed'):
            adapter.publish(make_command(), guard=denied)
        denied.assert_called_once()
        self.assertEqual(pub._sequence, 1)
        self.assertEqual(pub.sent, [])
        adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub.sent, [oracle(make_command(), pub.stamp, pub._node_name, 1)])

    def test_cold_body_capacity_covers_header_presence_and_fixed_wire_extremes(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        for node in ('', '机器人\x00SDK', '机器人' * 257):
            with self.subTest(node_bytes=len(node.encode('utf-8'))):
                pub = Publisher(node_name=node)
                adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
                self.addCleanup(adapter.close)
                capacity = adapter._header.body_capacity_bytes
                self.assertGreater(capacity, 0)
                command = dataclasses.replace(
                    make_command(), session_id=2**64-1, sequence=2**64-1,
                    input_state_id=2**64-1, input_state_ts_real_us=2**64-1,
                    ts_client_created_us=2**64-1, ts_client_send_us=2**64-1,
                    runtime_id=2**64-1, control_id=2**64-1,
                    action=14, root_quat=(1., -0., 0., .5))
                for stamp in (0, 2**63-1):
                    for sequence in (0, 1):
                        pub.stamp, pub._sequence = stamp, sequence
                        trace = {}
                        adapter.publish(command, trace, guard=lambda send: send())
                        self.assertEqual(pub.sent[-1], oracle(command, stamp, node, sequence))
                        self.assertEqual(trace['encode_buffer_capacity_bytes'], capacity)
                        self.assertEqual(trace['encode_buffer_allocations'], 1)
                        self.assertGreaterEqual(capacity, len(pub.sent[-1]))
                        self.assertEqual(adapter._header.body_capacity_bytes, capacity)

    def test_cold_node_layout_and_policy_do_not_follow_later_python_mutation(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher(node_name='captured-μ')
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        self.addCleanup(adapter.close)
        capacity = adapter._header.body_capacity_bytes
        pub._node_name = 'late-name' * 1000
        pub._include_node = False
        adapter.publish(make_command(), guard=lambda send: send())
        self.assertEqual(pub.sent, [oracle(make_command(), pub.stamp, 'captured-μ', 0)])
        self.assertEqual(adapter._header.body_capacity_bytes, capacity)
        with self.assertRaises(AttributeError):
            adapter._header.body_capacity_bytes = 1

    def test_cold_body_layout_does_not_admit_a_different_joint_count(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        self.addCleanup(adapter.close)
        capacity = adapter._header.body_capacity_bytes
        command = make_command()
        with self.assertRaisesRegex(ValueError, '31-slot tuple'):
            adapter.publish(dataclasses.replace(command, joints=command.joints + (BodyJointCmd(),)),
                            guard=lambda send: send())
        self.assertEqual(pub.sent, [])
        self.assertEqual(adapter._header.body_capacity_bytes, capacity)
        adapter.publish(command, guard=lambda send: send())
        self.assertEqual(pub.sent, [oracle(command, pub.stamp, pub._node_name, 1)])

    def test_transport_close_frees_header_context_before_node_close(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        from aorta.errors import InternalError, NotFoundError
        for free_error in (0, -5):
            with self.subTest(free_error=free_error):
                pub = Publisher()
                adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
                pub.free_error = free_error
                transport = object.__new__(_AortaTransport)
                transport._closed = False
                transport._info_lock = threading.Lock()
                transport._body_lock = threading.Lock()
                transport._info_clients = {}
                transport._body_publisher = adapter
                observed = []
                transport.node = SimpleNamespace(close=lambda: observed.append(tuple(pub.freed)))
                if free_error:
                    with self.assertRaises(InternalError):
                        transport.close()
                else:
                    transport.close()
                transport.close()
                self.assertEqual(pub.freed, [1])
                self.assertEqual(observed, [(1,)])
                with self.assertRaises(NotFoundError):
                    adapter.publish(make_command(), guard=lambda send: send())
                self.assertFalse(pub.sent)

    def test_header_construction_failure_preserves_primary_and_root_cleanup(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        binding = pub.native_header_binding()
        binding.close.side_effect = OSError('root cleanup failed')
        with mock.patch.object(pub, 'native_header_binding', return_value=binding), \
                mock.patch.object(native, 'HeaderContext', side_effect=ValueError('context create failed')):
            with self.assertRaisesRegex(ValueError, 'context create failed') as raised:
                NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        self.assertIsInstance(raised.exception.__cause__, OSError)
        binding.close.assert_called_once_with()
        self.assertFalse(pub.contexts)
        self.assertFalse(pub.sent)

    def test_root_cleanup_failure_consumes_clone_and_preserves_first_error(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        from aorta.errors import InternalError
        pub = Publisher()
        binding = pub.native_header_binding()
        binding.close.side_effect = OSError('root cleanup failed')
        pub.free_error = -5
        with mock.patch.object(pub, 'native_header_binding', return_value=binding):
            with self.assertRaisesRegex(OSError, 'root cleanup failed') as raised:
                NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        self.assertIsInstance(raised.exception.__cause__, InternalError)
        binding.close.assert_called_once_with()
        self.assertEqual(pub.freed, [1])
        self.assertFalse(pub.contexts)
        self.assertFalse(pub.sent)

    def test_schema_mismatch_fails_before_publish(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        with self.assertRaisesRegex(RuntimeError, 'pinned BodyCmd3'):
            NativeBodyPublisher(pub, SimpleNamespace(SCHEMA_HASH_HEX='wrong', SCHEMA_BFBS=b''))
        self.assertFalse(pub.sent)

    def test_missing_public_header_binding_has_no_private_field_fallback(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        publisher = SimpleNamespace(publish_bytes=mock.Mock())
        with self.assertRaisesRegex(RuntimeError, 'public native_header_binding'):
            NativeBodyPublisher(publisher, importlib.import_module('body_cmd_schema_meta'))
        publisher.publish_bytes.assert_not_called()

    def test_old_native_body2_cannot_pass_matching_metadata_guard(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        meta = importlib.import_module('body_cmd_schema_meta')
        with mock.patch.object(native, 'BODY_CMD_PROTOCOL_VERSION', 2):
            with self.assertRaisesRegex(RuntimeError, 'BodyCmd3 contract'):
                NativeBodyPublisher(pub, meta)
        self.assertFalse(pub.sent)

    def test_native_body3_rejects_missing_identity_and_wrong_wire_version(self):
        command = make_command()
        for invalid in (dataclasses.replace(command, runtime_id=None),
                        dataclasses.replace(command, runtime_id=0),
                        dataclasses.replace(command, runtime_id=True),
                        dataclasses.replace(command, control_id=0),
                        dataclasses.replace(command, control_id=True),
                        dataclasses.replace(command, protocol_version=2)):
            with self.subTest(command=invalid), self.assertRaises(ValueError):
                native.encode(invalid, 0, None, 0)

    def test_parallel_encode_owned_buffers_and_header_counter(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher();adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        with ThreadPoolExecutor(4) as pool:list(pool.map(lambda i:adapter.publish(make_command(i), guard=lambda send: send()), range(100)))
        body = importlib.import_module('lowlevel.BodyCmd').BodyCmd
        self.assertEqual(sorted(body.GetRootAs(b).AortaHeader().Sequence() for b in pub.sent), list(range(100)))
        self.assertEqual(len(pub.sent), 100)

    def test_transport_raw_route_profiles_failures_and_no_stale_trace(self):
        transport = object.__new__(_AortaTransport)
        transport._closed = False;transport._publish_trace = threading.local()
        transport._capture_publish_timing = True
        pub = Publisher();transport.publishers = {BODY_CMD_TOPIC: pub}
        transport._bindings = SimpleNamespace(body_cmd_schema_meta=importlib.import_module('body_cmd_schema_meta'))
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        transport._body_publisher = NativeBodyPublisher(pub, transport._bindings.body_cmd_schema_meta)
        transport._publish_body(make_command(), lambda send: send())
        trace = transport._last_publish_timing()
        self.assertIn('header_reserve_ns', trace)
        self.assertIn('send_call_wall_ns', trace)
        pub.failure = OSError('send')
        with self.assertRaises(OSError):transport._publish_body(make_command(), lambda send: send())
        self.assertEqual(transport._last_publish_timing()['header_sequence'], 1)
        transport._closed = True
        with self.assertRaises(RuntimeError):transport._publish_body(make_command(), lambda send: send())
        self.assertIsNone(transport._last_publish_timing())


class PublishTimingTest(unittest.TestCase):
    def harness(self):
        with mock.patch.dict(os.environ, {'LOCOMOTION_AORTA_PUBLISH_TIMING':'1'}):h=ControllerHarness()
        self.addCleanup(h.controller.close);h.accept_enter();h.controller.enter_external(.1)
        return h

    def test_returned_key_validation_failure_and_send_error_trace(self):
        h=self.harness();sent=h.controller.publish(make_command(), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        trace=h.controller.publish_timing
        self.assertEqual((trace['session_id'],trace['sequence']), (sent.session_id,sent.sequence))
        self.assertEqual(trace['outcome'], 'returned')
        for field in ['condition_wait_ns','validation_ns','session_replace_ns','transport_call_wall_ns','publish_total_ns']:
            self.assertGreaterEqual(trace[field],0)
        bad=dataclasses.replace(make_command(),joints=(BodyJointCmd(q=True),)*31)
        with self.assertRaises(InvalidCommandError):h.controller.publish(bad, target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        trace=h.controller.publish_timing;self.assertNotIn('session_id',trace)
        self.assertEqual(trace['error_type'],'InvalidCommandError')
        def fail(topic,message):
            if topic==BODY_CMD_TOPIC:raise OSError('send failed')
        h.transport.on_publish=fail
        with self.assertRaises(OSError):h.controller.publish(make_command(), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        trace=h.controller.publish_timing;self.assertEqual(trace['outcome'],'raised')
        self.assertIn('session_id',trace);self.assertEqual(trace['error_type'],'OSError')

    def test_diagnostic_getter_cannot_change_success_or_mask_send_failure(self):
        h = self.harness()
        h.transport._last_publish_timing = lambda: (_ for _ in ()).throw(ValueError('diagnostic'))
        sent = h.controller.publish(make_command(), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)
        self.assertEqual(h.controller.publish_timing['sequence'], sent.sequence)
        self.assertEqual(h.controller.publish_timing['timing_error_type'], 'ValueError')
        def fail(topic, message):
            if topic == BODY_CMD_TOPIC:
                raise OSError('actual send failure')
        h.transport.on_publish = fail
        with self.assertRaisesRegex(OSError, 'actual send failure'):
            h.controller.publish(make_command(), target_names=_BODY_TEST_TARGETS, required_fields=0x1f)

    def test_thread_local_profiles_do_not_cross_threads(self):
        h=self.harness()
        def publish(i):
            sent=h.controller.publish(make_command(i), target_names=_BODY_TEST_TARGETS, required_fields=0x1f);trace=h.controller.publish_timing
            return sent.sequence,trace['sequence']
        with ThreadPoolExecutor(4) as pool:keys=list(pool.map(publish,range(40)))
        self.assertTrue(all(a==b for a,b in keys));self.assertEqual(len({a for a,b in keys}),40)
        self.assertIsNone(h.controller.publish_timing)
