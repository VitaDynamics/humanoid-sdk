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
    def __init__(self):
        self._seq_lock = threading.Lock()
        self._sequence = 0
        self._closed = False
        self._include_node = True
        self._node_name = 'test-node'
        self._lib = SimpleNamespace(aorta_now_ns=self.clock)
        self.stamp, self.sent = 1234567, []
        self.failure = None

    def clock(self, ptr):
        ctypes.cast(ptr, ctypes.POINTER(ctypes.c_int64))[0] = self.stamp
        return 0

    def publish_bytes(self, data):
        if self.failure:
            raise self.failure
        self.sent.append(data)


@unittest.skipIf(native is None, 'installed native codec required; pkg must run this suite without skips')
class NativeBodyCmdTest(unittest.TestCase):
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
        for stem in ('body_cmd', 'body_state'):
            header = root / 'demo_onnx/cpp/body_cmd_vendor' / (stem + '_generated.h')
            self.assertEqual(hashlib.sha256(header.read_bytes()).hexdigest(), provenance[stem + '_generated_h_sha256'])
            self.assertEqual(provenance[stem + '_bfbs_sha256'], compatibility['schema_sha256'][stem])

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
        pub = Publisher();adapter = NativeBodyPublisher(pub, importlib.import_module('body_cmd_schema_meta'))
        for node in [True, False]:
            pub._include_node = node;seq = pub._sequence;adapter.publish(make_command(), guard=lambda send: send())
            self.assertEqual(pub.sent[-1], oracle(make_command(), pub.stamp,
                                                 pub._node_name if node else None, seq))
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

    def test_schema_mismatch_fails_before_publish(self):
        from locomotion_aorta._native_body_codec import NativeBodyPublisher
        pub = Publisher()
        with self.assertRaisesRegex(RuntimeError, 'pinned BodyCmd3'):
            NativeBodyPublisher(pub, SimpleNamespace(SCHEMA_HASH_HEX='wrong', SCHEMA_BFBS=b''))
        self.assertFalse(pub.sent)

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
        self.assertIn('core_clock_call_wall_ns', trace)
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
