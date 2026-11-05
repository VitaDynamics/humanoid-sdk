"""Native feedback parity and ownership; synthetic wire, no nodes or robot IO."""
import dataclasses
import hashlib
import importlib
import json
import math
from pathlib import Path
import random
import struct
import unittest
from unittest.mock import patch

import flatbuffers

from locomotion_aorta import _end_effector_codec as codec
from locomotion_aorta.end_effector_types import (
    EndEffectorState, EndEffectorTouch, InhibitReason, RegionStamp,
)
from test_end_effector_codec import module, pack, state_object, touch_object

try:
    from locomotion_aorta import _body_cmd_native as native
except ImportError:
    native = None


@unittest.skipIf(native is None, 'installed native feedback codec required; Pkg requires zero skips')
class NativeEndEffectorFeedbackTest(unittest.TestCase):
    def same(self, expected, actual):
        self.assertIs(type(actual), type(expected))
        if dataclasses.is_dataclass(expected):
            for field in dataclasses.fields(expected):
                with self.subTest(field=field.name):
                    self.same(getattr(expected, field.name), getattr(actual, field.name))
        elif isinstance(expected, tuple):
            self.assertEqual(len(actual), len(expected))
            for a, b in zip(expected, actual):
                self.same(a, b)
        elif isinstance(expected, float):
            if math.isnan(expected):
                self.assertTrue(math.isnan(actual))
            else:
                self.assertEqual(struct.pack('<d', actual), struct.pack('<d', expected))
        else:
            self.assertEqual(actual, expected)

    def view(self, kind, data):
        name = 'EndEffector' + kind.capitalize()
        return getattr(module(name), name).GetRootAs(data)

    def compare(self, kind, obj):
        data = pack(obj)
        view = self.view(kind, data)
        expected = getattr(codec, '_decode_' + kind + '_python')(view)
        actual = getattr(codec, 'decode_' + kind)(view)
        self.same(expected, actual)
        self.same(expected, getattr(codec, 'decode_' + kind)(data))
        self.same(expected.sample_ref, actual.sample_ref)
        return actual

    def test_state_all_fields_uint_edges_signed_zero_nan_and_axis_order(self):
        for seed in range(16):
            rng = random.Random(seed)
            obj = state_object(n=(1, 6, 7, 12)[seed % 4], side=1 + seed % 2,
                               sample=2**64 - 1, runtime=2**64 - 1)
            obj.layoutId = 2**32 - 1
            obj.tsStateRealUs = rng.randrange(2**64)
            obj.tsStatePubUs = rng.randrange(2**64)
            obj.tsAcquiredHwUs = rng.randrange(2**64)
            obj.tsIngestedUs = rng.randrange(2**64)
            obj.tsAssembledUs = rng.randrange(2**64)
            obj.sampleStatus = seed % 4
            obj.ready = bool(seed & 1)
            obj.outputAllowed = bool(seed & 2)
            obj.inhibitReason = seed % 5
            obj.touchEnabled = bool(seed & 4)
            obj.statusValidMask = seed
            values = (0., -0., 2**-149, -2**-149, math.nan, math.inf, -math.inf)
            for i, axis in enumerate(obj.axes):
                axis.validFields = (0, 1, 0xffffffff)[i % 3]
                axis.errorCode = (0, 0x80000000, 0xffffffff)[i % 3]
                for j, name in enumerate(('pos', 'vel', 'tau', 'current', 'temperatureC')):
                    setattr(axis, name, values[(seed + i + j) % len(values)])
            with self.subTest(seed=seed):
                self.compare('state', obj)

    def test_touch_all_bytes_and_independent_region_identity_time_and_flags(self):
        for side in (1, 2):
            obj = touch_object(side=side, sample=2**64 - 1, runtime=2**64 - 1)
            obj.layoutId = 2**32 - 1
            obj.tsStatePubUs = 2**64 - 1
            obj.publishTimeValid = False
            obj.payload = list(range(256)) * 8
            for i, region in enumerate(obj.regionStamps):
                region.sampleSeq = 2**32 - 1 - i
                region.tsRequestUs = i
                region.tsAcquiredHwUs = 2**64 - 1
                region.tsSourceUs = 0
                region.tsIngestedUs = 2**63 + i
                region.statusFlags = 0xffff - i
                region.directionFlags = 0xffff - i
                region.validity = 0xff if i == 0 else 0
            with self.subTest(side=side):
                result = self.compare('touch', obj)
                self.assertIs(type(result.payload), bytes)
                self.assertEqual(result.payload, bytes(range(256)) * 8)
                self.assertEqual(result.sample_ref.source_time_us, 0)

    def test_touch_absent_vectors_and_empty_payload_preserve_original_defaults(self):
        for regions, payload in ((None, None), ([], [])):
            obj = touch_object()
            obj.regionStamps, obj.payload = regions, payload
            result = self.compare('touch', obj)
            self.assertEqual((result.region_stamps, result.payload), ((), b''))

    def test_state_missing_or_empty_axes_and_wrong_protocol_keep_rejection(self):
        for axes, protocol in ((None, 2), ([], 2), (None, 0), ([], 3)):
            obj = state_object()
            obj.axes, obj.protocolVersion = axes, protocol
            view = self.view('state', pack(obj))
            for decoder in (codec._decode_state_python, codec.decode_state):
                with self.subTest(axes=axes, protocol=protocol, decoder=decoder):
                    with self.assertRaises(ValueError):
                        decoder(view)

    def test_touch_protocol_and_zero_runtime_stay_for_existing_validation_gate(self):
        for version in (0, 1, 2, 0xffffffff):
            obj = touch_object(runtime=0)
            obj.protocolVersion = version
            with self.subTest(version=version):
                self.compare('touch', obj)

    def test_state_defaults_preserve_unknown_times_and_same_id_permission_changes(self):
        for allowed in (True, False):
            obj = state_object(sample=123)
            obj.tsStateRealUs = obj.tsStatePubUs = obj.tsAcquiredHwUs = 0
            obj.tsIngestedUs = obj.tsAssembledUs = 0
            obj.outputAllowed, obj.touchEnabled = allowed, allowed
            obj.statusValidMask = 15
            result = self.compare('state', obj)
            self.assertEqual(result.state_id, 123)
            self.assertIs(result.output_allowed, allowed)
            self.assertEqual(result.sample_ref.source_time_us, 0)

    def test_embedded_table_view_uses_supplied_offset_not_the_outer_root(self):
        for kind, obj in (('state', state_object()), ('touch', touch_object())):
            builder = flatbuffers.Builder(1)
            child = obj.Pack(builder)
            builder.StartObject(1)
            builder.PrependUOffsetTRelativeSlot(0, child, 0)
            builder.Finish(builder.EndObject())
            data = bytes(builder.Output())
            parent = flatbuffers.table.Table(data, struct.unpack_from('<I', data)[0])
            slot = parent.Offset(4)
            table = parent.Indirect(parent.Pos + slot)
            cls = getattr(module('EndEffector' + kind.capitalize()), 'EndEffector' + kind.capitalize())
            view = cls()
            view.Init(data, table)
            with self.subTest(kind=kind):
                self.same(getattr(codec, '_decode_' + kind + '_python')(view),
                          getattr(codec, 'decode_' + kind)(view))

    def test_mutable_and_borrowed_input_never_escape_in_public_dto(self):
        for kind, obj in (('state', state_object()), ('touch', touch_object())):
            expected = self.compare(kind, obj)
            for buffer in (bytearray(pack(obj)), memoryview(bytearray(pack(obj)))):
                result = getattr(codec, 'decode_' + kind)(buffer)
                buffer[:] = b'\0' * len(buffer)
                with self.subTest(kind=kind, buffer=type(buffer)):
                    self.same(expected, result)

    def test_touch_region_constructor_reentry_cannot_change_later_regions_or_payload(self):
        obj = touch_object()
        data = bytearray(pack(obj))
        expected = codec._decode_touch_python(self.view('touch', bytes(data)))
        original = RegionStamp.__init__

        def initialize(stamp, *args, **kwargs):
            data[:] = b'\0' * len(data)
            original(stamp, *args, **kwargs)

        with patch.object(RegionStamp, '__init__', initialize):
            result = codec.decode_touch(self.view('touch', data))
        self.same(expected, result)

    def test_native_path_does_not_call_per_axis_or_per_byte_python_accessors(self):
        state = self.view('state', pack(state_object(n=6)))
        touch = self.view('touch', pack(touch_object()))
        with patch.object(type(state), 'Axes', side_effect=AssertionError('Python axis getter')):
            self.assertIs(type(codec.decode_state(state)), EndEffectorState)
        with patch.object(type(touch), 'Payload', side_effect=AssertionError('Python byte getter')):
            self.assertIs(type(codec.decode_touch(touch)), EndEffectorTouch)

    def test_existing_client_waits_and_same_sample_permission_revocation_keep_contract(self):
        from locomotion_aorta.end_effector import EndEffectorClient
        from test_controller import ControllerHarness
        from test_end_effector_client import Node, confirm_info
        from test_end_effector_codec import info_object

        harness = ControllerHarness(runtime_id=100)
        self.addCleanup(harness.controller.close)
        node = Node()
        harness.transport.node = node
        client = EndEffectorClient('left', connection=harness.controller, read_only=True)
        self.addCleanup(client.close)
        confirm_info(client, node, info_object(touch=True))
        obj = state_object(sample=123)
        node.emit('left', 'state', obj)
        before = client.wait_state(.01)
        self.assertIs(type(before), EndEffectorState)
        self.assertTrue(before.output_allowed)
        obj.outputAllowed = obj.touchEnabled = False
        obj.inhibitReason = InhibitReason.PASSIVE
        obj.tsStatePubUs += 1
        node.emit('left', 'state', obj)
        after = client.wait_state(.01)
        self.assertEqual(after.state_id, before.state_id)
        self.assertEqual(after.ts_state_real_us, before.ts_state_real_us)
        self.assertFalse(after.output_allowed)
        node.emit('left', 'touch', touch_object())
        touch = client.wait_touch(.01)
        self.assertIs(type(touch), EndEffectorTouch)
        self.assertIs(type(touch.payload), bytes)
        self.assertEqual(struct.unpack('<4H', touch.payload), (0, 0x7fff, 0x8000, 0xffff))
        client.close()
        self.assertEqual(touch.payload, struct.pack('<4H', 0, 0x7fff, 0x8000, 0xffff))
        self.assertEqual(node.commands, [])

    def test_truncated_roots_and_payload_fail_before_native_access(self):
        for kind, obj in (('state', state_object()), ('touch', touch_object())):
            valid = pack(obj)
            for data in (b'', b'\0', b'\0' * 4, b'\xff' * 16, valid[:len(valid) // 2]):
                with self.subTest(kind=kind, length=len(data)), self.assertRaises(ValueError):
                    getattr(codec, 'decode_' + kind)(data)
        data = bytearray(pack(touch_object()))
        view = self.view('touch', data)
        vector = view._tab.Vector(view._tab.Offset(22))
        struct.pack_into('<I', data, vector - 4, 0xffffffff)
        with self.assertRaises(ValueError):
            codec.decode_touch(data)

    def test_out_of_bounds_or_misaligned_table_offsets_reject(self):
        for kind, obj in (('state', state_object()), ('touch', touch_object())):
            data = pack(obj)
            decode = getattr(native, 'decode_end_effector_' + kind)
            for offset in (1, 2, len(data), len(data) + 4):
                with self.subTest(kind=kind, offset=offset), self.assertRaises(ValueError):
                    decode(data, offset)

    def test_wrong_native_version_or_schema_has_no_python_fallback(self):
        self.addCleanup(codec._feedback_decoder.cache_clear)
        for kind in ('state', 'touch'):
            prefix = 'EE_' + kind.upper()
            for attribute, value in ((prefix + '_PROTOCOL_VERSION', 999),
                                     (prefix + '_SCHEMA_SHA256', '0' * 64)):
                codec._feedback_decoder.cache_clear()
                with patch.object(native, attribute, value), self.subTest(attribute=attribute):
                    with self.assertRaisesRegex(RuntimeError, 'requires the pinned schema'):
                        codec._feedback_decoder(kind)

    def test_metadata_bfbs_hash_corruption_is_rejected_before_native_decode(self):
        self.addCleanup(codec._feedback_decoder.cache_clear)
        for kind in ('state', 'touch'):
            codec._feedback_decoder.cache_clear()
            meta = importlib.import_module('end_effector_' + kind + '_schema_meta')
            with patch.object(meta, 'SCHEMA_BFBS', b'wrong'), self.subTest(kind=kind):
                with self.assertRaisesRegex(ValueError, 'metadata does not match'):
                    codec._feedback_decoder(kind)

    def test_real_generated_header_hashes_and_bfbs_match_pinned_messages(self):
        root = Path(__file__).resolve().parents[1]
        vendor = root / 'demo_onnx/cpp/end_effector_vendor'
        provenance = json.loads((vendor / 'provenance.json').read_text())
        compatibility = json.loads((root / 'compatibility.json').read_text())
        self.assertEqual(provenance['wheel_version'], compatibility['humanoid_msgs_version'])
        wheel = 'vbot_humanoid_msgs-' + provenance['wheel_version'] + '-py3-none-any.whl'
        self.assertEqual(provenance['wheel_sha256'], compatibility['wheel_sha256'][wheel])
        for name, expected in provenance['headers_sha256'].items():
            self.assertEqual(hashlib.sha256((vendor / name).read_bytes()).hexdigest(), expected)
        for name, expected in provenance['shared_body_vendor_include_sha256'].items():
            self.assertEqual(hashlib.sha256((root / 'demo_onnx/cpp/body_cmd_vendor' / name).read_bytes()).hexdigest(), expected)
        for kind in ('state', 'touch'):
            expected = compatibility['schema_sha256']['end_effector_' + kind]
            self.assertEqual(provenance['bfbs_sha256']['end_effector_' + kind], expected)
            self.assertEqual(getattr(native, 'EE_' + kind.upper() + '_SCHEMA_SHA256'), expected)
