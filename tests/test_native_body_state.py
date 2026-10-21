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
                 missing_sample=False, protocol=2):
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
                 'AppliedSessionId', 'AppliedSequence', 'TsFirstOutputUs'):
        getattr(m, 'BodyStateAdd' + name)(builder, rng.randrange(2**64))
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
        actual, timing = decode_body_state_native(view, measure=True)
        self.same(expected, actual)
        self.assertTrue(timing['gil_released_during_decode'])
        for name in ('decode_input_own_ns', 'decode_native_ns',
                     'decode_gil_reacquire_wall_ns', 'decode_python_objects_ns'):
            self.assertGreaterEqual(timing[name], 0)
        return actual

    def test_original_generated_golden_all_fields_and_imu_order(self):
        data, _ = make_state()
        self.compare(data)

    def test_seeded_random_golden_256_full_uint_unknown_nan_and_clocks(self):
        for seed in range(256):
            with self.subTest(seed=seed): self.compare(state_buffer(seed))

    def test_absent_joints_imus_and_optional_sample_keep_defaults(self):
        for kwargs in ({'missing_joints':True}, {'missing_imus':True}, {'missing_sample':True}):
            with self.subTest(**kwargs): self.compare(state_buffer(3, **kwargs))
        b = flatbuffers.Builder(1);m = module('BodyState')
        m.BodyStateStart(b);m.BodyStateAddProtocolVersion(b, 2);b.Finish(m.BodyStateEnd(b))
        state = self.compare(bytes(b.Output()))
        self.assertEqual((state.joints, state.imu_states), ((), ()))
        self.assertEqual((state.state_id, state.applied_session_id, state.applied_sequence), (0, 0, 0))

    def test_wrong_protocol_and_present_wrong_joint_lengths_reject(self):
        for kwargs in ({'protocol':0}, {'protocol':1}, {'protocol':3},
                       {'count':0}, {'count':30}, {'count':32}, {'count':42}):
            view = module('BodyState').BodyState.GetRootAs(state_buffer(1, **kwargs))
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError): _decode_body_state_python(view)
                with self.assertRaises(ValueError): decode_body_state(view)

    def test_truncated_corrupt_and_invalid_table_offsets_reject(self):
        data = state_buffer(3);pos = struct.unpack_from('<I', data)[0]
        for n in range(len(data)):
            with self.subTest(length=n), self.assertRaises(ValueError):
                native.decode_state(data[:n], pos)
        for off in (0, 1, 2, 3, len(data), len(data)+1, 2**63):
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

    def test_parallel_decodes_have_owned_independent_state_and_release_gil(self):
        payloads = [state_buffer(i) for i in range(32)]
        with ThreadPoolExecutor(4) as pool:
            states = list(pool.map(self.compare, payloads))
        self.assertEqual(len({id(s) for s in states}), 32)

    def test_native_entry_legacy_snapshot_and_schema_pin(self):
        data = state_buffer(3);view = module('BodyState').BodyState.GetRootAs(data)
        with mock.patch('locomotion_aorta._body_codec._decode_body_state_python', side_effect=AssertionError('oracle used')):
            self.same(decode_body_state(view), BodyStateSnapshot(data, module('BodyState').BodyState.GetRootAs, 1, 2).body_state())
        _state_decoder.cache_clear()
        with mock.patch('body_state_schema_meta.SCHEMA_HASH_HEX', 'wrong'):
            with self.assertRaisesRegex(RuntimeError, 'pinned BODY2'): decode_body_state(view)
        _state_decoder.cache_clear()
        self.compare(data)

    def test_unmeasured_native_path_has_no_diagnostic_fields(self):
        data = state_buffer(3);view = module('BodyState').BodyState.GetRootAs(data)
        state, timing = native.decode_state(data, view._tab.Pos)
        self.assertIsNone(timing);self.same(_decode_body_state_python(view), state)

