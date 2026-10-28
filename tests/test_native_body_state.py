"""Native/pinned-Python state goldens; no Aorta nodes or control requests."""
import dataclasses
import importlib
import math
import random
import struct
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest import mock

import flatbuffers
from locomotion_aorta import BodyStateSnapshot
from locomotion_aorta._body_codec import _decode_body_state_python
from locomotion_aorta._native_body_codec import decode_body_state_native, _state_decoder
decode_body_state = decode_body_state_native
from test_body_codec import make_state, module

try:
    from locomotion_aorta import _body_cmd_native as native
except ImportError:
    native = None


def state_buffer(seed, *, count=31, missing_joints=False, missing_imus=False,
                 missing_sample=False, protocol=3):
    rng = random.Random(seed)
    builder = flatbuffers.Builder(1)
    imu_module = module('BodyImuSample')
    imus = []
    floats = [0., -0., 2**-149, -2**-149, math.nan, math.inf, -math.inf]
    floats += [rng.uniform(-100, 100) for _ in range(16)]
    for i in range(seed % 5):
        imu_module.BodyImuSampleStart(builder)
        imu_module.BodyImuSampleAddLocation(builder, [0, 1, 2, 255][i % 4])
        imu_module.BodyImuSampleAddStatus(builder, [0, 1, 7, 255][i % 4])
        for name in ('TsSampledUs', 'TsAcquiredHwUs', 'TsIngestedUs', 'TsAssembledUs'):
            getattr(imu_module, 'BodyImuSampleAdd' + name)(builder, rng.randrange(2**64))
        if not missing_sample:
            groups = [tuple(rng.choice(floats) for _ in range(n)) for n in (4, 3, 3, 3)]
            imu_module.BodyImuSampleAddState(builder, module('BodyImuState').CreateBodyImuState(builder, *groups))
        imus.append(imu_module.BodyImuSampleEnd(builder))
    m = module('BodyState')
    m.BodyStateStartImuStatesVector(builder, len(imus))
    for v in reversed(imus): builder.PrependUOffsetTRelative(v)
    imu_vector = builder.EndVector()
    m.BodyStateStartJointsVector(builder, count)
    for _ in range(count):
        module('BodyJointState').CreateBodyJointState(builder,
            *(rng.randrange(2**64) for _ in range(3)),
            *(rng.choice(floats) for _ in range(3)), rng.choice([0, 8, 255, 0x80000000, 0xffffffff]))
    joints = builder.EndVector()
    m.BodyStateStart(builder)
    m.BodyStateAddProtocolVersion(builder, protocol)
    m.BodyStateAddRosterVersion(builder, rng.randrange(2**32))
    for name in ('StateId', 'TsStateRealUs', 'TsStatePubUs', 'TsConsumedUs',
                 'AppliedSessionId', 'AppliedSequence', 'TsFirstOutputUs', 'TsControlUs'):
        getattr(m, 'BodyStateAdd' + name)(builder, rng.randrange(2**64))
    ref = importlib.import_module('lowlevel.ee.DescriptorRef')
    m.BodyStateAddDescriptorRef(builder, ref.CreateDescriptorRef(builder, 2**64-1))
    m.BodyStateAddAppliedKeyValid(builder, bool(seed % 2))
    m.BodyStateAddAssociationSourceType(builder, seed % 256)
    m.BodyStateAddAttitudeValid(builder, bool(seed % 2))
    if not missing_joints: m.BodyStateAddJoints(builder, joints)
    if not missing_imus: m.BodyStateAddImuStates(builder, imu_vector)
    builder.Finish(m.BodyStateEnd(builder))
    return bytes(builder.Output())


@unittest.skipIf(native is None, 'installed native state codec required; pkg requires zero skips')
class NativeBodyStateTest(unittest.TestCase):
    def same(self, expected, actual):
        self.assertIs(type(expected), type(actual))
        if dataclasses.is_dataclass(expected):
            for f in dataclasses.fields(expected):
                with self.subTest(field=f.name): self.same(getattr(expected, f.name), getattr(actual, f.name))
        elif isinstance(expected, tuple):
            self.assertEqual(len(expected), len(actual))
            for a, b in zip(expected, actual): self.same(a, b)
        elif isinstance(expected, float):
            if math.isnan(expected): self.assertTrue(math.isnan(actual))
            else: self.assertEqual(struct.pack('<d', expected), struct.pack('<d', actual))
        else: self.assertEqual(expected, actual)

    def compare(self, data, *, offset=0):
        view = module('BodyState').BodyState.GetRootAs(data, offset)
        expected = _decode_body_state_python(view)
        actual = decode_body_state_native(view)
        self.same(expected, actual)
        return actual

    def test_original_generated_golden_all_fields_and_imu_order(self):
        data, _ = make_state()
        self.compare(data)

    def test_seeded_random_golden_256_full_uint_unknown_nan_and_clocks(self):
        for seed in range(256):
            with self.subTest(seed=seed): self.compare(state_buffer(seed))

    def test_absent_joints_imus_and_optional_sample_keep_defaults(self):
        for kwargs in ({'missing_imus':True}, {'missing_sample':True}):
            with self.subTest(**kwargs): self.compare(state_buffer(3, **kwargs))
        missing = state_buffer(3, missing_joints=True)
        with self.assertRaisesRegex(ValueError, '31 joint slots'):
            _decode_body_state_python(module('BodyState').BodyState.GetRootAs(missing))
        with self.assertRaisesRegex(ValueError, '31 joint slots'):
            decode_body_state_native(missing)
        b = flatbuffers.Builder(1);m = module('BodyState')
        m.BodyStateStart(b);m.BodyStateAddProtocolVersion(b, 3);b.Finish(m.BodyStateEnd(b))
        with self.assertRaisesRegex(ValueError, '31 joint slots'):
            self.compare(bytes(b.Output()))

    def test_wrong_protocol_and_present_wrong_joint_lengths_reject(self):
        for kwargs in ({'protocol':0}, {'protocol':1}, {'protocol':2}, {'protocol':4}, {'protocol':0xffffffff},
                       {'count':0}, {'count':30}, {'count':32}, {'count':42}):
            view = module('BodyState').BodyState.GetRootAs(state_buffer(1, **kwargs))
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError): _decode_body_state_python(view)
                with self.assertRaises(ValueError): decode_body_state(view)

    def test_valid_zero_key_all_source_association_reaches_native_dto(self):
        m = module('BodyState')
        ref = importlib.import_module('lowlevel.ee.DescriptorRef')
        for source in (0, 1, 2, 3, 4, 5, 6, 255):
            with self.subTest(source=source):
                obj = m.BodyStateT.InitFromObj(make_state()[1])
                obj.descriptorRef = ref.DescriptorRefT(2**64-1)
                obj.tsControlUs, obj.appliedKeyValid, obj.associationSourceType = 2**64-2, True, source
                b = flatbuffers.Builder(1); b.Finish(obj.Pack(b))
                state = self.compare(bytes(b.Output()))
                self.assertEqual((state.applied_session_id, state.applied_sequence, state.ts_first_output_us), (0, 0, 0))
                self.assertTrue(state.applied_key_valid)
                self.assertEqual(state.association_source_type, source)
                self.assertEqual(state.descriptor_ref.robot_runtime_id, 2**64-1)
                self.assertEqual(state.ts_control_us, 2**64-2)

    def test_present_zero_runtime_descriptor_is_rejected_by_python_and_native(self):
        m = module('BodyState')
        ref = importlib.import_module('lowlevel.ee.DescriptorRef')
        obj = m.BodyStateT.InitFromObj(make_state()[1])
        obj.descriptorRef = ref.DescriptorRefT(0)
        builder = flatbuffers.Builder(1)
        builder.Finish(obj.Pack(builder))
        payload = bytes(builder.Output())
        with self.assertRaisesRegex(ValueError, 'nonzero robot_runtime_id'):
            _decode_body_state_python(m.BodyState.GetRootAs(payload))
        with self.assertRaisesRegex(ValueError, 'nonzero robot_runtime_id'):
            native.decode_state(payload, 0)

    def test_truncated_corrupt_and_invalid_table_offsets_reject(self):
        data = state_buffer(3);pos = struct.unpack_from('<I', data)[0]
        for n in range(len(data)):
            with self.subTest(length=n), self.assertRaises(ValueError):
                native.decode_state(data[:n], pos)
        for off in (1, 2, 3, len(data), len(data)+1, 2**63):
            with self.subTest(offset=off), self.assertRaises(ValueError): native.decode_state(data, off)
        corrupt = bytearray(data);struct.pack_into('<i', corrupt, pos, 0x7fffffff)
        with self.assertRaises(ValueError): native.decode_state(corrupt, pos)

    def test_bytes_bytearray_memoryview_embedded_offset_and_owned_results(self):
        data = state_buffer(3);expected = self.compare(data)
        for value in (data, bytearray(data), memoryview(data), memoryview(bytearray(data))):
            self.same(expected, self.compare(value))
        self.same(expected, self.compare(bytes(8)+data, offset=8))
        source = bytearray(data);result = decode_body_state(module('BodyState').BodyState.GetRootAs(source))
        source[:] = bytes(len(source));self.same(expected, result)

    def test_parallel_decodes_have_owned_independent_state_with_gil_held(self):
        payloads = [state_buffer(i) for i in range(32)]
        with ThreadPoolExecutor(4) as pool:
            states = list(pool.map(self.compare, payloads))
        self.assertEqual(len({id(s) for s in states}), 32)

    def test_production_pull_dto_uses_native_and_matches_python_field_by_field(self):
        from test_pull import PullTest, S
        from locomotion_aorta import PullController
        from locomotion_aorta.transport import BODY_STATE_TOPIC
        t = PullTest().transport()
        controller = PullController.__new__(PullController)
        controller.transport = t
        controller._condition = __import__("threading").Condition()
        controller._closed = False
        _state_decoder.cache_clear()
        self.addCleanup(_state_decoder.cache_clear)
        with mock.patch.object(native, 'decode_state', wraps=native.decode_state) as decode:
            for seed in range(32):
                data = state_buffer(seed)
                t._pulls[BODY_STATE_TOPIC].samples.append(
                    S(payload=data, context=S(receive_stamp_ns=42)))
                with mock.patch('locomotion_aorta.pull.decode_body_state_arrays',
                                side_effect=AssertionError('array materialization used')):
                    actual = controller.wait_body_state(.1)
                expected = _decode_body_state_python(module('BodyState').BodyState.GetRootAs(data))
                self.same(expected, actual)
                self.assertIs(controller.wait_body_state(0), actual)
                self.assertEqual(decode.call_count, seed + 1)
                decode.assert_called_with(data, 0)
        _state_decoder.cache_clear()
        with mock.patch('body_state_schema_meta.SCHEMA_HASH_HEX', 'wrong'):
            with self.assertRaisesRegex(RuntimeError, 'pinned BodyState3'):
                decode_body_state_native(data)
        _state_decoder.cache_clear()

    def test_bad_pull_frame_raises_once_and_preserves_previous_snapshot(self):
        from test_pull import PullTest, S
        from locomotion_aorta.transport import BODY_STATE_TOPIC
        for arrays in (True, False):
            with self.subTest(arrays=arrays):
                t = PullTest().transport()
                read = t.wait_state if arrays else t._wait_body_state
                ring = t._pulls[BODY_STATE_TOPIC]
                def put(payload):
                    ring.samples.append(S(payload=payload, context=S(receive_stamp_ns=42)))
                put(state_buffer(3))
                first = read(0)
                receipt = t._latest_sample
                for bad in (b'bad', state_buffer(3, missing_joints=True),
                            state_buffer(3, count=30)):
                    put(bad)
                    with self.assertRaises(ValueError):
                        read(0)
                    self.assertIs(t._latest_sample, receipt)
                    self.assertIs(read(0), first)
                put(state_buffer(4))
                self.assertNotEqual(read(0).state_id, first.state_id)
                empty = PullTest().transport()
                empty._pulls[BODY_STATE_TOPIC].samples.append(
                    S(payload=b'bad', context=S(receive_stamp_ns=42)))
                empty_read = empty.wait_state if arrays else empty._wait_body_state
                with self.assertRaises(ValueError): empty_read(0)
                with self.assertRaises(TimeoutError): empty_read(0)

    def test_stale_native_protocol_cannot_pass_even_with_matching_old_metadata(self):
        _state_decoder.cache_clear()
        self.addCleanup(_state_decoder.cache_clear)
        with mock.patch.object(native, 'BODY_STATE_PROTOCOL_VERSION', 2):
            with self.assertRaisesRegex(RuntimeError, 'pinned BodyState3'):
                decode_body_state_native(state_buffer(3))
        _state_decoder.cache_clear()

    def test_native_entry_returns_only_the_state_dto(self):
        data = state_buffer(3);view = module('BodyState').BodyState.GetRootAs(data)
        state = native.decode_state(data, view._tab.Pos)
        self.same(_decode_body_state_python(view), state)

