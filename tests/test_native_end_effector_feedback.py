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
from test_native_body_cmd import Publisher
from locomotion_aorta.end_effector_types import (
    CommandKey, EndEffectorSubmission, EndEffectorTarget, Field, SampleRef,
)

try:
    from locomotion_aorta import _body_cmd_native as native
except ImportError:
    native = None


@unittest.skipIf(native is None, 'installed native codec required; Pkg requires zero skips')
class NativeEndEffectorCommandTest(unittest.TestCase):
    @staticmethod
    def command(n=1, sources=(), side=1):
        target = EndEffectorTarget((1,)*n, (0x3f,)*n, (.03125,)*n,
                                  (-.015625,)*n, (.125,)*n, (-.25,)*n, (.0625,)*n)
        samples = tuple(SampleRef(100, stream, sample_side, 100+i, 200+i, 7)
                        for i, (stream, sample_side) in enumerate(sources))
        return EndEffectorSubmission(100, 7, side, 13, CommandKey(5, 0), target, samples)

    def context(self, command, publisher=None):
        publisher = Publisher() if publisher is None else publisher
        binding = publisher.native_header_binding()
        context = native.EECommandContext(binding, command.runtime_id, command.layout_id,
            command.side, len(command.target.mode),
            tuple((s.stream, s.side) for s in command.input_observations))
        binding.close()
        self.addCleanup(context.close)
        return publisher, context

    def oracle(self, command, publisher, sequence):
        from aorta.header import build_aorta_header
        builder = flatbuffers.Builder(256)
        header = build_aorta_header(builder, publish_stamp_ns=publisher.stamp,
            publisher_node=publisher._node_name if publisher._include_node else None,
            sequence=sequence)
        builder.Finish(codec.fill_command(codec.load_bindings(), command, builder, header))
        return bytes(builder.Output())

    def test_real_cmd_root_matches_python_bytes_for_frozen_n_k_sides_and_headers(self):
        sources = ((1, 0), (2, 2), (3, 1), (2, 1), (3, 2))
        for n in (1, 6, 7, 12, 33):
            for selected in ((), sources[1:3], sources):
                for side, name in ((1, ''), (2, '机器人\x00EE')):
                    command = self.command(n, selected, side)
                    publisher = Publisher(node_name=name)
                    publisher.stamp = 0
                    _, context = self.context(command, publisher)
                    for stamp in (0, 123):
                        publisher.stamp = stamp
                        sequence = publisher._sequence
                        with self.subTest(n=n, sources=selected, side=side, node=name,
                                          stamp=stamp, sequence=sequence):
                            rc, payload, trace = context.encode(command, True)
                            self.assertEqual(rc, 0)
                            self.assertEqual(payload, self.oracle(command, publisher, sequence))
                            self.assertEqual(trace['encode_buffer_capacity_bytes'], context.capacity_bytes)
                            self.assertEqual(trace['encode_buffer_allocations'], 1)
                            self.assertEqual(trace['header_sequence'], sequence)
        self.assertEqual(native.EE_CMD_PROTOCOL_VERSION, 2)
        self.assertEqual(native.EE_CMD_SCHEMA_SHA256,
                         codec.load_bindings().metas['cmd'].SCHEMA_HASH_HEX)

    def test_frozen_identity_and_nonzero_control_fail_without_relabel_or_send(self):
        command = self.command()
        publisher, context = self.context(command)
        for changes in ({'runtime_id': 101}, {'runtime_id': True}, {'layout_id': 8},
                        {'side': 2}, {'control_id': 0}, {'protocol_version': 1}):
            with self.subTest(changes=changes), self.assertRaises((ValueError, TypeError)):
                context.encode(dataclasses.replace(command, **changes))
        self.assertEqual(publisher.sent, [])
        rc, payload, _ = context.encode(command)
        self.assertEqual(rc, 0)
        decoded = codec.decode_cmd(module('EndEffectorCmd').EndEffectorCmd.GetRootAs(payload))
        self.assertEqual((decoded.runtime_id, decoded.control_id, decoded.key), (100, 13, command.key))

    def test_local_sample_r_and_order_checked_before_compact_wire_projection(self):
        command = self.command(sources=((2, 2), (1, 0)))
        publisher, context = self.context(command)
        samples = command.input_observations
        for wrong in ((dataclasses.replace(samples[0], runtime_id=99), samples[1]),
                      tuple(reversed(samples)), samples[:1],
                      (dataclasses.replace(samples[0], validity=16), samples[1])):
            with self.assertRaises(ValueError):
                context.encode(dataclasses.replace(command, input_observations=wrong))
        selected = (dataclasses.replace(samples[0], sample_id=100, source_time_us=0, validity=0),
                    dataclasses.replace(samples[1], sample_id=102, source_time_us=999, validity=15))
        actual = dataclasses.replace(command, input_observations=selected)
        _, payload, _ = context.encode(actual)
        self.assertEqual(payload, self.oracle(actual, publisher, publisher._sequence-1))
        decoded = codec.decode_cmd(module('EndEffectorCmd').EndEffectorCmd.GetRootAs(payload))
        self.assertEqual(decoded.input_observations, selected)

    def test_header_busy_clock_error_and_negative_stamp_precede_command_getters(self):
        publisher, context = self.context(self.command())
        class Unreadable:
            @property
            def protocol_version(self):
                raise AssertionError('command getter ran after Header failure')
        for field, code, consumed in (('reserve_error', -9, 0), ('clock_error', -5, 1)):
            setattr(publisher, field, code)
            self.assertEqual(context.encode(Unreadable()), (code, None, None))
            self.assertEqual(publisher._sequence, consumed)
            setattr(publisher, field, 0)
        publisher.stamp = -1
        with self.assertRaises(TypeError):
            context.encode(Unreadable())
        self.assertEqual(publisher._sequence, 2)
        self.assertEqual(publisher.sent, [])

    def test_axis_column_shape_is_frozen_and_bad_payload_does_not_poison_next_call(self):
        command = self.command(2)
        publisher, context = self.context(command)
        for wrong in (dataclasses.replace(command.target, mode=(1,)),
                      dataclasses.replace(command.target, command_fields=(1,)),
                      dataclasses.replace(command.target, pos=(0.,)),
                      dataclasses.replace(command.target, tau=(math.inf, 0.))):
            with self.assertRaises(ValueError):
                context.encode(dataclasses.replace(command, target=wrong))
        _, payload, _ = context.encode(command)
        self.assertEqual(payload, self.oracle(command, publisher, publisher._sequence-1))
        with self.assertRaises(AttributeError):
            context.capacity_bytes = 1

    def test_python_float_getter_reentry_preserves_each_call_axes_and_header(self):
        outer = self.command(2)
        inner = dataclasses.replace(outer, key=CommandKey(6, 9),
            target=dataclasses.replace(outer.target, pos=(.01, .02)))
        publisher, context = self.context(outer)
        nested = []
        class Reenter:
            def __float__(self):
                nested.append(context.encode(inner)[1])
                return .03125
        with_reentry = dataclasses.replace(outer,
            target=dataclasses.replace(outer.target, pos=(Reenter(), .03125)))
        _, payload, _ = context.encode(with_reentry)
        self.assertEqual(payload, self.oracle(outer, publisher, 0))
        self.assertEqual(nested, [self.oracle(inner, publisher, 1)])

    def test_public_clone_close_and_closed_publisher_do_not_read_or_send(self):
        publisher, context = self.context(self.command())
        self.assertEqual(context.close(), 0)
        self.assertEqual(context.close(), 0)
        self.assertEqual(len(publisher.freed), 1)
        self.assertEqual(context.encode(object()), (-2, None, None))
        other, live = self.context(self.command())
        other._closed = True
        self.assertEqual(live.encode(object()), (-2, None, None))
        self.assertEqual(other.sent, [])

    def test_selected_zero_empty_columns_and_float32_edges_keep_wire_bits(self):
        command = self.command(2)
        publisher, context = self.context(command)
        for first in (0., -0., 2**-149, -2**-149, .03125):
            current = dataclasses.replace(command, target=dataclasses.replace(
                command.target, pos=(first, .02), vel=(), current=(),
                command_fields=tuple(mask & ~(Field.VEL | Field.CURRENT)
                                     for mask in command.target.command_fields)))
            _, payload, _ = context.encode(current)
            self.assertEqual(payload, self.oracle(current, publisher, publisher._sequence-1))

    def test_cold_bad_layout_releases_header_clone_without_reserving_a_sequence(self):
        for n, sources in ((0, ()), (1, ((2, 1), (2, 1))), (1, ((1, 2),))):
            publisher = Publisher()
            binding = publisher.native_header_binding()
            with self.assertRaises(ValueError):
                native.EECommandContext(binding, 100, 7, 1, n, sources)
            self.assertEqual(publisher._sequence, 0)
            self.assertEqual(len(publisher.freed), 1)
            self.assertEqual(publisher.sent, [])


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
        vendor = root / 'demo_onnx/cpp/body_cmd_vendor'
        provenance = json.loads((root / 'demo_onnx/cpp/end_effector_vendor/provenance.json').read_text())
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
