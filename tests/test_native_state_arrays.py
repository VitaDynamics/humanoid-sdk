"""Pinned Python-vs-native NumPy goldens; no nodes, control or model IO."""
import gc
import importlib.util
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest import mock
from locomotion_aorta.state_arrays import decode_body_state_arrays
import test_native_body_state as golden
native, state_buffer = golden.native, golden.state_buffer
from test_body_codec import module, make_state
from locomotion_aorta._body_codec import decode_body_state

@unittest.skipIf(native is None or importlib.util.find_spec('numpy') is None,
                 'installed native codec and state-arrays extra required; installed gate must not skip')
class NativeStateArraysTest(unittest.TestCase):
    same = golden.NativeBodyStateTest.same

    def compare(self, data):
        expected = decode_body_state(module('BodyState').BodyState.GetRootAs(data))
        actual = decode_body_state_arrays(data)
        self.same(expected, actual.body_state())
        return actual

    def test_seeded_256_full_field_numpy_goldens_and_original_fixture(self):
        self.compare(make_state()[0])
        for seed in range(256):
            with self.subTest(seed=seed): self.compare(state_buffer(seed))

    def test_dtypes_shapes_uint64_error_uint32_and_no_float_integer_rounding(self):
        import numpy as np
        v=self.compare(state_buffer(3))
        self.assertEqual(v.q.dtype,np.dtype('float32'))
        self.assertEqual(v.joint_stamps_us.dtype,np.dtype('uint64'))
        self.assertEqual(v.error_code.dtype,np.dtype('uint32'))
        self.assertEqual(v.imu_stamps_us.dtype,np.dtype('uint64'))
        self.assertEqual(v.q.shape,(31,));self.assertEqual(v.joint_stamps_us.shape,(31,3))
        self.assertEqual(v.imu_values.shape,(3,13));self.assertEqual(v.imu_stamps_us.shape,(3,4))
        self.assertGreater(int(v.joint_stamps_us.max()),2**53)
        self.assertEqual(v.stamp_ns,v.ts_state_real_us*1000)

    def test_absent_vectors_optional_imu_unknown_enums_and_nan(self):
        for kw in ({'missing_joints':True},{'missing_imus':True},{'missing_sample':True}):
            data=state_buffer(4,**kw);v=self.compare(data)
            if kw.get('missing_sample'):self.assertFalse(v.imu_state_present.any())
        self.assertIn(255,self.compare(state_buffer(4)).imu_location)

    def test_owned_readonly_contiguous_arrays_survive_input_and_result_deletion(self):
        import numpy as np
        data=bytearray(state_buffer(3));v=self.compare(data);saved=v.q.copy();q=v.q
        data[:]=bytes(len(data));del v;gc.collect()
        np.testing.assert_array_equal(q,saved)
        self.assertTrue(q.flags.c_contiguous);self.assertFalse(q.flags.writeable)
        with self.assertRaises(ValueError):q[0]=1

    def test_parallel_results_do_not_share_mutable_cache_storage(self):
        import numpy as np
        with ThreadPoolExecutor(4)as pool:states=list(pool.map(self.compare,[state_buffer(i)for i in range(32)]))
        self.assertFalse(np.shares_memory(states[0].q,states[1].q))

    def test_protocol_length_truncation_and_schema_guard_errors(self):
        for kw in ({'protocol':0},{'protocol':3},{'count':30},{'count':32}):
            with self.assertRaises(ValueError):decode_body_state_arrays(state_buffer(3,**kw))
        for data in (b'',b'abc',state_buffer(3)[:-1]):
            with self.assertRaises(ValueError):decode_body_state_arrays(data)
        from locomotion_aorta._native_body_codec import _state_decoder
        _state_decoder.cache_clear()
        with mock.patch('body_state_schema_meta.SCHEMA_HASH_HEX','wrong'):
            with self.assertRaisesRegex(RuntimeError,'pinned BODY2'):decode_body_state_arrays(state_buffer(3))
        _state_decoder.cache_clear()

    def test_compatibility_materialization_is_lazy(self):
        v=decode_body_state_arrays(state_buffer(3))
        self.assertNotIn('joints',v.__dict__);self.assertNotIn('imu_states',v.__dict__)
        self.assertEqual(len(v.joints),31);self.assertIs(v.joints,v.joints)
        self.assertEqual(v.received_monotonic_ns,0)

    def test_wbc_policy_reads_arrays_without_materializing_joint_dtos(self):
        import numpy as np
        from test_wbc_tracking_policy import policy
        data,view=make_state()
        expected=policy().state(decode_body_state(view))
        state=decode_body_state_arrays(data)
        actual=policy().state(state)
        for a,b in zip(expected,actual):np.testing.assert_array_equal(a,b)
        self.assertNotIn('joints',state.__dict__)
