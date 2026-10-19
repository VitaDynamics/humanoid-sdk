"""Generated-accessor oracles, layout negatives and immutable column ownership."""
import dataclasses
import gc
import math
import struct
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest import mock

from decode_fixtures import state_object, pack_state
from test_body_codec import make_state, module
from locomotion_aorta import BodyStateSnapshot, FrameToken, BodyImuLocation
from locomotion_aorta import _body_codec as codec, _state_view as selected


def snapshot(data=None, sequence=1):
    return BodyStateSnapshot(make_state()[0] if data is None else data,
                             module('BodyState').BodyState.GetRootAs,
                             FrameToken('test', sequence), 111, 222)


def assert_value(test, actual, expected):
    if isinstance(expected, float) and math.isnan(expected):
        test.assertTrue(math.isnan(actual))
    elif dataclasses.is_dataclass(expected):
        for field in dataclasses.fields(expected):
            assert_value(test, getattr(actual, field.name), getattr(expected, field.name))
    elif isinstance(expected, (tuple, list)):
        test.assertEqual(len(actual), len(expected))
        for a, e in zip(actual, expected): assert_value(test, a, e)
    else:
        test.assertEqual(actual, expected)


class StateViewTest(unittest.TestCase):
    def oracle(self, data, indices=tuple(range(31)), fields=('q', 'dq')):
        frame = snapshot(data)
        view = module('BodyState').BodyState.GetRootAs(data)
        columns = frame.select(joint_indices=indices, fields=fields).columns()
        self.assertEqual(columns.token, frame.token)
        self.assertEqual((columns.received_monotonic_ns, columns.received_realtime_ns), (111, 222))
        self.assertEqual(columns.joint_indices, indices)
        self.assertEqual(tuple(columns.values), fields)
        for field in dataclasses.fields(columns.metadata):
            self.assertEqual(getattr(columns.metadata, field.name), getattr(view, codec._getter(field.name))())
        for name in ('body_index', 'populated', 'is_mock', 'status', 'valid_fields',
                     'ts_acquired_hw_us', 'ts_ingested_us', 'ts_assembled_us', 'lost') + fields:
            actual = columns.values[name] if name in fields else getattr(columns, name)
            self.assertTrue(actual.readonly)
            expected = [getattr(view.Joints(i), codec._getter(name))() for i in indices] if view.JointsLength() else []
            assert_value(self, actual.tolist(), expected)
        return frame, columns

    def test_all_columns_subset_order_and_legal_nan_match_actual_accessors(self):
        for indices in (tuple(range(31)), (30, 0, 14, 2), (), (14,)):
            for fields in (('q', 'dq'), ('dq', 'q'), ('q',), ('dq',)):
                with self.subTest(indices=indices, fields=fields):
                    self.oracle(make_state()[0], indices, fields)

    def test_root_order_padding_defaults_unknown_field_and_shared_vtables(self):
        for reverse in (False, True):
            for defaults in (False, True):
                for prefix in (0, 3, 31):
                    obj = state_object(applied=True)
                    data = pack_state(obj, reverse_root=reverse, defaults=defaults, prefix=prefix)
                    frame, _ = self.oracle(data, (30, 14, 0), ('dq', 'q'))
                    full = codec.decode_body_state(module('BodyState').BodyState.GetRootAs(data))
                    assert_value(self, frame.body_state(), full)
                    assert_value(self, frame.applied_cmd(), full.applied_cmd)

    def test_negative_root_vtable_delta_and_shared_imu_vtables(self):
        import copy
        obj=state_object()
        obj.imuStates=[copy.copy(obj.imuStates[1]) for _ in range(10)]
        for i,imu in enumerate(obj.imuStates):imu.location=10+i
        data=bytearray(pack_state(obj))
        view=module('BodyState').BodyState.GetRootAs(data)
        self.assertTrue(any(struct.unpack_from('<i',data,view.ImuStates(i)._tab.Pos)[0]<0
                            for i in range(10)))
        pos=view._tab.Pos
        old=pos-struct.unpack_from('<i',data,pos)[0]
        length=struct.unpack_from('<H',data,old)[0]
        relocated=len(data)
        data.extend(data[old:old+length])
        struct.pack_into('<i',data,pos,pos-relocated)
        self.assertLess(struct.unpack_from('<i',data,pos)[0],0)
        frame,_=self.oracle(bytes(data))
        self.assertEqual(frame.select(joint_indices=(),fields=('q',),imu_location=15).imu_sample().location,15)
        self.assertEqual(len(frame.body_state().imu_states),10)

    def test_unread_broken_imu_does_not_force_full_decode(self):
        data=bytearray(make_state()[0])
        view=module('BodyState').BodyState.GetRootAs(data)
        field=view._tab.Pos+view._tab.Offset(20)
        struct.pack_into('<I',data,field,0xfffffff0)
        frame=snapshot(bytes(data))
        frame.select(joint_indices=(0,),fields=('q',)).columns()
        with self.assertRaises(Exception):frame.select(joint_indices=(),fields=('q',),imu_location=1).imu_sample()
        with self.assertRaises(Exception):frame.body_state()

    def test_every_mask_clock_status_and_body_index_preserved_without_health_filter(self):
        obj = state_object()
        for i, j in enumerate(obj.joints):
            j.bodyIndex = 30-i
            j.isMock = bool(i%2)
            j.populated = bool(i%3)
            j.status = 255-i
            j.validFields = 65535-i
            j.tsAcquiredHwUs = (1<<64)-1-i
            j.tsIngestedUs = (1<<63)+i
            j.tsAssembledUs = i
            j.lost = (1<<32)-1-i
            j.q = math.inf if i==0 else -math.inf if i==1 else math.nan if i==14 else i+.5
        self.oracle(pack_state(obj))

    def test_missing_joints_retains_empty_instead_of_fabricating_zero(self):
        for joints in (None,):
            obj = state_object();obj.joints = joints
            frame, cols = self.oracle(pack_state(obj))
            self.assertEqual(len(cols.values['q']), 0)
            self.assertEqual(frame.body_state().joints, ())

    def test_invalid_selection_arguments(self):
        frame = snapshot()
        for indices in ([1], (1,1), (31,), (-1,), (True,), ('1',)):
            with self.subTest(indices=indices), self.assertRaises(ValueError):
                frame.select(joint_indices=indices, fields=('q',))
        for fields in ((), ('q','q'), ('tau_est',), ['q'], (True,)):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                frame.select(joint_indices=(0,), fields=fields)
        for location in (True, 'HIP', 1.5):
            with self.assertRaises(ValueError):
                frame.select(joint_indices=(0,), fields=('q',), imu_location=location)

    def test_imu_explicit_location_missing_null_invalid_and_ambiguity(self):
        obj = state_object()
        obj.imuStates[1].state = None
        obj.imuStates[1].validFields = 0
        obj.imuStates[1].status = 255
        obj.imuStates[1].tsSampledUs = 987
        frame = snapshot(pack_state(obj))
        hip = frame.select(joint_indices=(), fields=('q',), imu_location=BodyImuLocation.HIP).imu_sample()
        full = frame.body_state()
        assert_value(self, hip, full.imu_states[1])
        self.assertIsNone(hip.state)
        self.assertIsNone(frame.select(joint_indices=(), fields=('q',)).imu_sample())
        self.assertIsNone(frame.select(joint_indices=(), fields=('q',), imu_location=999).imu_sample())
        obj.imuStates.append(obj.imuStates[1])
        frame = snapshot(pack_state(obj))
        request = frame.select(joint_indices=(0,), fields=('q',), imu_location=BodyImuLocation.HIP)
        request.columns()  # Duplicate IMU does not invalidate an unrequested component.
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            request.imu_sample()
        self.assertIsNotNone(frame.select(joint_indices=(), fields=('q',), imu_location=BodyImuLocation.TRUNK).imu_sample())

    def test_optional_applied_echo_keeps_independent_source_and_unknown_default(self):
        self.assertIsNone(snapshot().applied_cmd())
        for action in (13, 255):
            obj = state_object(applied=True);obj.appliedCmd.action=action
            obj.appliedCmd.sourceKeyValid=False
            frame = snapshot(pack_state(obj))
            echo = frame.applied_cmd()
            assert_value(self, echo, frame.body_state().applied_cmd)
            self.assertEqual((echo.session_id, echo.sequence, echo.cmd_id), (781,992,123))
            self.assertFalse(echo.source_key_valid)
            self.assertEqual(echo.action, action)
            self.assertIs(echo, frame.applied_cmd())

    def test_malformed_unread_applied_is_lazy_but_requested_component_raises(self):
        data = bytearray(pack_state(state_object(applied=True)))
        view = module('BodyState').BodyState.GetRootAs(data)
        echo = view.AppliedCmd()
        field = echo._tab.Pos + echo._tab.Offset(14)
        vector = field + struct.unpack_from('<I', data, field)[0]
        struct.pack_into('<I', data, vector, 30)
        frame = snapshot(bytes(data))
        frame.select(joint_indices=(0,), fields=('q',)).columns()
        self.assertEqual(frame.metadata.state_id, 91)
        with self.assertRaises(ValueError): frame.applied_cmd()
        with self.assertRaises(ValueError): frame.body_state()

    def test_malformed_joint_count_alignment_root_offset_and_truncation_match_eager_errors(self):
        data, view = make_state()
        field = view._tab.Pos + view._tab.Offset(22)
        vector = field + struct.unpack_from('<I', data, field)[0]
        variants = [data[:4], data[:vector+4+56*31-1], b'bad']
        for count in (0, 30, 32, 0xffffffff):
            b=bytearray(data);struct.pack_into('<I',b,vector,count);variants.append(bytes(b))
        for offset in (1, 3, 0xffffffff):
            b=bytearray(data);struct.pack_into('<I',b,field,offset);variants.append(bytes(b))
        b=bytearray(data);struct.pack_into('<I',b,0,0xfffffffc);variants.append(bytes(b))
        for bad in variants:
            with self.subTest(length=len(bad)):
                frame = snapshot(bad)
                with self.assertRaises(Exception):frame.select(joint_indices=(0,), fields=('q',)).columns()
                with self.assertRaises(Exception):frame.body_state()

    def test_concurrent_success_only_cache_and_no_cross_frame_reuse(self):
        frame = snapshot()
        original = selected._columns
        with mock.patch.object(selected, '_columns', wraps=original) as gather:
            with ThreadPoolExecutor(max_workers=8) as pool:
                columns=list(pool.map(lambda _: frame.select(joint_indices=(1,14),fields=('q','dq')).columns(),range(32)))
            self.assertEqual(gather.call_count,1)
            self.assertTrue(all(c is columns[0] for c in columns))
            frame.select(joint_indices=(14,1),fields=('q','dq')).columns()
            frame.select(joint_indices=(1,14),fields=('dq','q')).columns()
            self.assertEqual(gather.call_count,3)
            obj=state_object();obj.joints[1].q=123
            new=snapshot(pack_state(obj),sequence=2).select(joint_indices=(1,14),fields=('q','dq')).columns()
            self.assertEqual(new.values['q'][0],123)
            self.assertEqual(columns[0].values['q'][0],1.125)

    def test_readonly_bytes_ownership_gc_and_view_release_independence(self):
        frame=snapshot()
        columns=frame.select(joint_indices=(0,14),fields=('q','dq')).columns()
        a=columns.values['q'];b=columns.values['q'];mask=columns.populated
        for view in (a,b,mask):
            self.assertTrue(view.readonly)
            self.assertIsInstance(view.obj,bytes)
            with self.assertRaises(TypeError):view[0]=7
        with self.assertRaises(TypeError):columns.values['q']=memoryview(b'bad')
        a.release();mask.release()
        self.assertEqual(b[0],.125)
        self.assertEqual(columns.populated.tolist(),[1,0])
        del frame,columns
        gc.collect()
        self.assertEqual(b[0],.125)

    def test_decode_failure_is_not_a_success_cache_entry(self):
        frame=snapshot()
        original=selected._columns
        with mock.patch.object(selected,'_columns',side_effect=[ValueError('bad'),original(frame,(0,),('q',))]) as gather:
            request=frame.select(joint_indices=(0,),fields=('q',))
            with self.assertRaises(ValueError):request.columns()
            self.assertEqual(request.columns().values['q'][0],.125)
            self.assertEqual(gather.call_count,2)

    def test_local_token_does_not_normalize_repeated_wrapped_or_rebooted_state_id(self):
        for seq,state_id in enumerate((0,(1<<64)-1,0,0,1),1):
            obj=state_object();obj.stateId=state_id
            frame=snapshot(pack_state(obj),seq)
            self.assertEqual(frame.token.sequence,seq)
            self.assertEqual(frame.metadata.state_id,state_id)
            self.assertEqual(frame.body_state().state_id,state_id)


if __name__ == '__main__': unittest.main()
