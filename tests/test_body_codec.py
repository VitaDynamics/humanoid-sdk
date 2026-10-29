"""Real generated public factories/accessors; no device, no native transport."""
import dataclasses
import importlib
import math
import struct
import unittest
from types import SimpleNamespace
import flatbuffers
from locomotion_aorta import (
    BodyCmd, BodyJointCmd, BodyCmdAction, BodySampleStatus, BodyImuLocation,
    BodySlotRole, ServiceStatus, BodyStateSnapshot,
)
from locomotion_aorta import _body_codec as codec
from locomotion_aorta.transport import _fill_body_cmd

BODY_RUNTIME_ID = 11


def module(name):
    return importlib.import_module('lowlevel.' + name)


def make_command(seed=0):
    return BodyCmd(tuple(BodyJointCmd(i + seed + .125, -i - .25, i * .5, i + 1., .75)
                            for i in range(31)), action=BodyCmdAction.EXTERNAL,
                  session_id=42, sequence=7, input_state_id=123, input_state_ts_real_us=456,
                  ts_client_created_us=789, ts_client_send_us=901,
                  runtime_id=11, control_id=13)


def encode(command, *, generated=False, initial_size=1, prefix=0):
    builder = flatbuffers.Builder(initial_size)
    if prefix:
        builder.CreateByteVector(bytes(prefix))
    m = module('BodyCmd')
    if generated:
        joint = module('BodyJointCmd')
        m.BodyCmdStartJointsVector(builder, len(command.joints))
        for value in reversed(command.joints):
            joint.CreateBodyJointCmd(builder, value.q, value.dq, value.tau, value.kp, value.kd)
        vector = builder.EndVector()
        m.BodyCmdStart(builder)
        m.BodyCmdAddAortaHeader(builder, 0)
        m.BodyCmdAddJoints(builder, vector)
        builder.ForceDefaults(True)
        m.BodyCmdAddProtocolVersion(builder, command.protocol_version)
        m.BodyCmdAddSessionId(builder, command.session_id)
        m.BodyCmdAddSequence(builder, command.sequence)
        m.BodyCmdAddInputStateId(builder, command.input_state_id)
        m.BodyCmdAddInputStateTsRealUs(builder, command.input_state_ts_real_us)
        m.BodyCmdAddTsClientCreatedUs(builder, command.ts_client_created_us)
        m.BodyCmdAddTsClientSendUs(builder, command.ts_client_send_us)
        m.BodyCmdAddAction(builder, command.action)
        quat = command.root_quat or (1., 0., 0., 0.)
        m.BodyCmdAddRootQuat(builder, module('RootQuatWxyz').CreateRootQuatWxyz(builder, *quat))
        m.BodyCmdAddRuntimeId(builder, command.runtime_id)
        m.BodyCmdAddControlId(builder, command.control_id)
        root = m.BodyCmdEnd(builder)
        builder.ForceDefaults(False)
    else:
        root = _fill_body_cmd(SimpleNamespace(body_cmd_module=m), command, builder, 0)
    builder.Finish(root)
    return bytes(builder.Output())


def make_state(*, error_codes=None, runtime_id=None):
    builder = flatbuffers.Builder(1)
    # All IMUs retain clocks/rpy; deliberately use HEAD,TRUNK,HIP order.
    imu_module = module('BodyImuSample')
    imus = []
    for location in (BodyImuLocation.HEAD, BodyImuLocation.TRUNK, BodyImuLocation.HIP):
        imu_module.BodyImuSampleStart(builder)
        imu_module.BodyImuSampleAddLocation(builder, location)
        imu_module.BodyImuSampleAddState(builder, module('BodyImuState').CreateBodyImuState(
            builder, (1., 0., 0., 0.), (.1, .2, .3), (0., 0., 9.81), (-1., 2., 3.)))
        imu_module.BodyImuSampleAddStatus(builder, BodySampleStatus.OK)
        imu_module.BodyImuSampleAddTsSampledUs(builder, 101)
        imu_module.BodyImuSampleAddTsAcquiredHwUs(builder, 102)
        imu_module.BodyImuSampleAddTsIngestedUs(builder, 103)
        imu_module.BodyImuSampleAddTsAssembledUs(builder, 104)
        imus.append(imu_module.BodyImuSampleEnd(builder))
    m = module('BodyState')
    m.BodyStateStartImuStatesVector(builder, len(imus))
    for imu in reversed(imus): builder.PrependUOffsetTRelative(imu)
    imus = builder.EndVector()
    cols = [tuple(range(31)), tuple(1000 + i for i in range(31)), tuple(2000 + i for i in range(31)),
            tuple(math.nan if i == 14 else i + .125 for i in range(31)),
            (-.25,) * 31, (-.75,) * 31, tuple(0xfffffff9 if i == 30 else 0 if i in (0,14,15) else 3 if i == 1 else 8 if i == 2 else 255 if i == 3 else i for i in range(31))]
    if error_codes is not None:
        cols[-1] = tuple(error_codes)
    m.BodyStateStartJointsVector(builder, 31)
    for values in reversed(tuple(zip(*cols))):
        module('BodyJointState').CreateBodyJointState(builder, *values)
    joints = builder.EndVector()
    m.BodyStateStart(builder)
    m.BodyStateAddProtocolVersion(builder, 3)
    m.BodyStateAddStateId(builder, 91)
    m.BodyStateAddTsStateRealUs(builder, 1_234_567)
    m.BodyStateAddTsStatePubUs(builder, 1_234_890)
    m.BodyStateAddImuStates(builder, imus)
    m.BodyStateAddAttitudeValid(builder, True)
    m.BodyStateAddJoints(builder, joints)
    m.BodyStateAddRuntimeId(builder, BODY_RUNTIME_ID if runtime_id is None else runtime_id)
    builder.Finish(m.BodyStateEnd(builder))
    data = bytes(builder.Output())
    return data, m.BodyState.GetRootAs(data)


def _minimal_joints_vector(builder):
    m = module('BodyState')
    joint = module('BodyJointState')
    m.BodyStateStartJointsVector(builder, 31)
    for _ in range(31):
        joint.CreateBodyJointState(builder, 0, 0, 0, 0., 0., 0., 0)
    return builder.EndVector()


class BodyCodecTest(unittest.TestCase):
    def test_plain_body_capability_valid_zero_and_unknown_limits(self):
        from locomotion_aorta import BodyJointCapability
        absent = BodyJointCapability(14, BodySlotRole.ACTIVE, True, 0, 0,
                                     math.nan, math.nan, 0., 0., 0., 0., 0., 0., 0., 0.)
        self.assertTrue(absent.is_mock)
        self.assertEqual(absent.valid_limits, 0)
        zero = BodyJointCapability(0, BodySlotRole.ACTIVE, False, 1, 1,
                                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0.)
        self.assertEqual((zero.q_min, zero.q_max), (0., 0.))
        with self.assertRaisesRegex(ValueError, 'invalid valid limit: q'):
            dataclasses.replace(zero, q_min=math.nan)
        with self.assertRaisesRegex(ValueError, 'invalid valid limit: q'):
            dataclasses.replace(zero, q_min=1.)

    def test_plain_endpoint_results_preserve_no_device_and_hardware_failure(self):
        from locomotion_aorta import EndEffectorCapability, RobotInfo
        no_device = EndEffectorCapability(1, True, None, False,
                                           ServiceStatus.UNAVAILABLE, 'no device')
        failed = EndEffectorCapability(2, False, None, False,
                                       ServiceStatus.INTERNAL_ERROR, 'initialization failed')
        info = RobotInfo(2, 0, None, (no_device, failed))
        self.assertTrue(info.end_effectors[0].is_mock)
        self.assertFalse(info.end_effectors[0].ready)
        self.assertFalse(info.end_effectors[1].is_mock)
        self.assertFalse(info.end_effectors[1].ready)
        self.assertEqual(info.end_effectors[1].reason, 'initialization failed')
        with self.assertRaisesRegex(ValueError, 'ordered LEFT and RIGHT'):
            RobotInfo(2, 0, None, (failed, no_device))
        with self.assertRaisesRegex(ValueError, 'boolean'):
            EndEffectorCapability(1, 1, None, False, ServiceStatus.UNAVAILABLE, 'no device')

    def test_robot_info_preserves_service_status_without_descriptor_confirmation(self):
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse
        info = module('RobotInfo').RobotInfoT(protocolVersion=2, tsInfoUs=987, endEffectors=[
            module('EndEffectorCapability').EndEffectorCapabilityT(
                side=side, isMock=True, ready=False, status=ServiceStatus.UNAVAILABLE,
                reason='no endpoint') for side in (1, 2)])
        reply = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS,
            runtimeId=11, info=info, errorMessage='right unavailable')
        builder = flatbuffers.Builder(1)
        builder.Finish(reply.Pack(builder))
        wire = GetRobotInfoResponse.GetRootAs(builder.Output())
        result = codec.decode_robot_info_response(wire)
        self.assertFalse(hasattr(result.info, 'committed_descriptor_ref'))
        self.assertEqual((result.status, result.error_message, result.info.protocol_version,
                          result.info.ts_info_us),
            (ServiceStatus.PARTIAL_SUCCESS, 'right unavailable', 2, 987))
        self.assertEqual(tuple(e.side for e in result.info.end_effectors), (1, 2))
        self.assertIsNone(result.info.body)

    def test_bulk_is_byte_identical_to_real_factory_with_growth_padding_and_actions(self):
        for size in (1, 7, 64, 4096):
            for prefix in (0, 1, 3, 17):
                for action in (BodyCmdAction.EXTERNAL, BodyCmdAction.HUMANOID_TRACKING, BodyCmdAction.UNKNOWN):
                    with self.subTest(size=size, prefix=prefix, action=action):
                        command = dataclasses.replace(make_command(prefix), action=action,
                                                      root_quat=(1., 0., 0., 0.) if action == BodyCmdAction.HUMANOID_TRACKING else None)
                        actual = encode(command, initial_size=size, prefix=prefix)
                        self.assertEqual(actual, encode(command, generated=True, initial_size=size, prefix=prefix))
                        view = module('BodyCmd').BodyCmd.GetRootAs(actual)
                        self.assertEqual(view.Action(), action)
                        self.assertNotEqual(view._tab.Offset(22), 0)  # action id9 is explicitly present
                        self.assertEqual(view.JointsLength(), 31)
                        self.assertEqual(view.Joints(30).Q(), command.joints[30].q)
                        self.assertEqual(view.RuntimeId(), 11)
                        self.assertEqual(view.ControlId(), 13)
                        self.assertIsNotNone(view.RootQuat())

    def test_compact_command_vector_id8_is_distinct_from_state_id9(self):
        # Construct the new table directly: this test can run before regenerating
        # BodyCmd bindings, and cannot accidentally inherit their former id9.
        from flatbuffers.table import Table
        command = make_command()
        builder = flatbuffers.Builder(1)
        builder.StartVector(20, 31, 4)
        for joint in reversed(command.joints):
            module('BodyJointCmd').CreateBodyJointCmd(
                builder, joint.q, joint.dq, joint.tau, joint.kp, joint.kd)
        vector = builder.EndVector()
        builder.StartObject(11)
        builder.PrependUOffsetTRelativeSlot(8, vector, 0)
        builder.PrependUint8Slot(9, BodyCmdAction.EXTERNAL, 0)
        builder.Finish(builder.EndObject())
        data = bytes(builder.Output())
        view = SimpleNamespace(_tab=Table(data, struct.unpack_from('<I', data)[0]))
        self.assertEqual(codec.unpack_body_cmd(view), command.joints)
        # The old shared id9 slot now contains the action scalar, not a vector.
        with self.assertRaises(ValueError):
            codec.unpack_body_cmd(view, slot=22)

    def test_no_vector_or_wrong_width_is_accepted_by_bulk(self):
        for count in (0, 30, 32, 42, 44):
            with self.subTest(count=count), self.assertRaises(ValueError):
                codec.create_body_cmd(flatbuffers.Builder(1), (BodyJointCmd(),) * count)

    def test_full_uint32_errors_match_generated_accessors_and_lazy_snapshots(self):
        for code in (0x7fffffff, 0x80000000, 0xffffffff):
            with self.subTest(error_code=code):
                data, view = make_state(error_codes=(code,) * 31)
                expected = tuple(view.Joints(i).ErrorCode() for i in range(31))
                self.assertEqual(expected, (code,) * 31)
                snapshot = BodyStateSnapshot(data, module('BodyState').BodyState.GetRootAs, 7, 9)
                for state in (codec.decode_body_state(view), snapshot.body_state()):
                    self.assertEqual(tuple(j.error_code for j in state.joints), expected)

    def test_state_matches_every_real_accessor_and_keeps_invalid_roll_and_clocks(self):
        data, view = make_state()
        state = codec.decode_body_state(view)
        self.assertEqual(len(state.joints), 31)
        self.assertEqual(state.ts_state_real_us, 1_234_567)
        self.assertEqual(state.ts_state_pub_us, 1_234_890)
        for i, joint in enumerate(state.joints):
            generated = view.Joints(i)
            for field in dataclasses.fields(joint):
                value = getattr(joint, field.name)
                expected = getattr(generated, codec._getter(field.name))()
                if isinstance(value, float) and math.isnan(value): self.assertTrue(math.isnan(expected))
                else: self.assertEqual(value, expected)
        self.assertEqual(state.joints[14].error_code, 0)  # Installation comes from BodyInfo.
        self.assertEqual(state.joints[0].error_code, 0)
        self.assertEqual(state.joints[30].error_code, 0xfffffff9)
        self.assertEqual([state.joints[i].error_code for i in (1,2,3,14,15)],
                         [3,8,255,0,0])  # Real fault, timeout, unknown, absent, MOCK.
        self.assertEqual(state.joints[0].tau_est, -.75)
        self.assertEqual((state.applied_session_id, state.applied_sequence, state.ts_first_output_us), (0,0,0))
        self.assertEqual([s.location for s in state.imu_states],
                         [BodyImuLocation.HEAD, BodyImuLocation.TRUNK, BodyImuLocation.HIP])
        self.assertEqual(state.imu_states[1].state.rpy, (-1., 2., 3.))
        sample = state.imu_states[1]
        self.assertEqual((sample.ts_sampled_us, sample.ts_acquired_hw_us,
                          sample.ts_ingested_us, sample.ts_assembled_us), (101, 102, 103, 104))

    def test_v1_is_rejected_before_any_fixed_stride_or_imu_decode(self):
        from types import SimpleNamespace
        # Only protocol_version is safe to read from the incompatible v1 frame.
        for version in (0, 1, 2, 4, 0xffffffff):
            with self.subTest(version=version), self.assertRaisesRegex(ValueError, 'protocol_version'):
                builder = flatbuffers.Builder(1)
                m = module('BodyState')
                m.BodyStateStart(builder)
                m.BodyStateAddProtocolVersion(builder, version)
                builder.Finish(m.BodyStateEnd(builder))
                codec.decode_body_state(m.BodyState.GetRootAs(builder.Output()))

    def test_joint_error_enum_and_removed_dto_fields_follow_public_bindings(self):
        from locomotion_aorta import JointError, BodyJointState, BodyImuSample
        expected = {'NONE': 0, 'OVER_VOLTAGE': 1, 'UNDER_VOLTAGE': 2, 'OVER_CURRENT': 3, 'OVER_TEMPERATURE': 4, 'OVERLOAD': 5, 'ENCODER_FAULT': 6, 'DRIVER_FAULT': 7, 'TIMEOUT': 8, 'ENABLE_REJECTED': 9, 'OVERLOAD_WARNING': 10, 'OVER_TEMP_WARNING': 11, 'JOINT_LIMIT_EXCEEDED': 12, 'STALL': 13, 'ENCODER_NOT_CALIBRATED': 14, 'CONTROL_FAULT': 15, 'MOTOR_PHASE_FAULT': 16, 'SENSOR_FAULT': 17, 'INVALID_PARAM': 30, 'UNKNOWN': 255}
        self.assertEqual({name: getattr(JointError, name) for name in expected}, expected)
        self.assertEqual({name for name in vars(JointError) if name.isupper()}, set(expected))
        self.assertEqual([f.name for f in dataclasses.fields(BodyJointState)],
                         ['ts_acquired_hw_us', 'ts_ingested_us',
                          'ts_assembled_us', 'q', 'dq', 'tau_est', 'error_code'])
        for cls in (BodyJointState, BodyImuSample):
            names = {f.name for f in dataclasses.fields(cls)}
            self.assertFalse(names & {'populated', 'is_mock', 'valid_fields', 'ddq', 'temperature_deg_c', 'lost'})
        self.assertFalse(hasattr(BodyJointState(), 'body_index'))
        self.assertFalse(hasattr(BodyJointState(), 'status'))
        self.assertFalse(hasattr(BodyImuSample(0, None, 0), 'device_timestamp_ms'))
        self.assertFalse(hasattr(BodyImuSample(0, None, 0), 'host_timestamp_us'))
        # Preserve unknown numeric errors; do not add an enum-range gate.
        sample = BodyJointState(error_code=123)
        self.assertEqual(sample.error_code, 123)

    def test_truncated_vector_payload_is_rejected(self):
        data, view = make_state()
        pos = view.Joints(0)._tab.Pos
        for missing in (1, 20, 40, 1240):
            short = data[:pos + 1240 - missing]
            with self.subTest(missing=missing), self.assertRaises((ValueError, struct.error)):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(short))

    def test_actual_generated_layout_and_table_slots_guard_bulk_assumptions(self):
        self.assertEqual(module('BodyJointCmd').BodyJointCmd.SizeOf(), codec._MOTOR.size)
        self.assertEqual(module('BodyJointState').BodyJointState.SizeOf(), codec._JOINT_STATE.size)
        class Probe:
            Pos = 0
            Bytes = bytes(2048)
            def __init__(self): self.positions = []
            def Offset(self, slot): self.positions.append(slot); return 0
            def Get(self, flags, pos): self.positions.append(int(pos)); return 0
        for name, getters, expected in (
            ('BodyJointState', [codec._getter(f.name) for f in dataclasses.fields(codec.BodyJointState)],
             [0,8,16,24,28,32,36]),
            ('BodyJointCmd', ['Q','Dq','Tau','Kp','Kd'], [0,4,8,12,16]),
            ('BodyState', ['JointsLength'], [22]),
            ('BodyImuSample', ['TsSampledUs', 'TsIngestedUs'], [4,8]),
            ('BodyState', ['AppliedSessionId', 'AppliedSequence', 'TsFirstOutputUs'], [24,26,28]),
            ('BodyState', ['RuntimeId', 'TsControlUs', 'AppliedKeyValid', 'AssociationSourceType'], [30,32,34,36]),
            ('BodyCmd', ['JointsLength','Action','RootQuat'], [20,22,24]),
        ):
            with self.subTest(table=name, getters=getters):
                view = getattr(module(name), name)(); probe = Probe(); view._tab = probe
                for getter in getters: getattr(view, getter)()
                self.assertEqual(probe.positions, expected)

    def _synthetic_endpoint_info(self, *, side, runtime, n=1, mock=True, layout=7):
        """Unified six-field endpoint Info fixture (synthetic, schema73)."""
        from lowlevel.ee.EndEffectorInfo import EndEffectorInfoT
        from lowlevel.ee.TouchRegionInfo import TouchRegionInfoT
        from lowlevel.ee.PositionDomain import PositionDomain as PD
        info = EndEffectorInfoT()
        info.protocolVersion = 1
        info.runtimeId = runtime
        info.layoutId = layout
        info.side = side
        info.deviceModel = 0
        info.firmware = 'synthetic'
        info.serial = 'synthetic'
        info.identityValidMask = 0
        info.axisNames = ['axis' + str(i) for i in range(n)]
        info.positionDomain = [PD.TOTAL_OPENING] * n
        info.torqueDomain = [0] * n
        info.supportedModes = [1 << 1] * n
        info.supportedCommandFields = [1] * n
        info.supportedStateFields = [1] * n
        info.validLimits = [1] * n
        info.posMin = [0.] * n
        info.posMax = [.0625] * n
        info.velMin = []
        info.velMax = []
        info.velLimitMax = []
        info.tauMin = []
        info.tauMax = []
        info.currentMin = []
        info.currentMax = []
        info.touchAvailable = False
        info.touchRegions = []
        info.isMock = [mock] * n
        info.tsDescriptorAcquiredUs = 987
        info.tsStatePubUs = 988
        info.timeValidMask = 3
        return info

    def test_robot_info_nested_decode_does_not_import_online_end_effector_codecs(self):
        import builtins
        from unittest.mock import patch
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse

        for nested in (True, False):
            with self.subTest(nested_tables=nested):
                left_info = self._synthetic_endpoint_info(side=1, runtime=11) if nested else None
                right_info = self._synthetic_endpoint_info(side=2, runtime=11) if nested else None
                info = module('RobotInfo').RobotInfoT(protocolVersion=2, tsInfoUs=1234, endEffectors=[
                    module('EndEffectorCapability').EndEffectorCapabilityT(
                        side=1, isMock=nested, info=left_info,
                        ready=False, status=ServiceStatus.UNAVAILABLE, reason='left pending'),
                    module('EndEffectorCapability').EndEffectorCapabilityT(
                        side=2, isMock=nested, info=right_info,
                        ready=False, status=ServiceStatus.PARTIAL_SUCCESS, reason='right diagnostic'),
                ])
                reply = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS, runtimeId=11,
                                             info=info, errorMessage='discovery incomplete')
                builder = flatbuffers.Builder(1)
                builder.Finish(reply.Pack(builder))
                wire = GetRobotInfoResponse.GetRootAs(builder.Output())
                original_import = builtins.__import__

                def data_only_import(name, *args, **kwargs):
                    if name.rsplit('.', 1)[-1] in ('_hand_codec', '_gripper_codec'):
                        raise AssertionError('Body discovery imported online codec: ' + name)
                    return original_import(name, *args, **kwargs)

                with patch('builtins.__import__', side_effect=data_only_import):
                    result = codec.decode_robot_info_response(wire)

                self.assertEqual(result.status, ServiceStatus.PARTIAL_SUCCESS)
                self.assertEqual(result.error_message, 'discovery incomplete')
                self.assertIsNone(result.info.body)
                left, right = result.info.end_effectors
                self.assertEqual((left.status, left.reason), (ServiceStatus.UNAVAILABLE, 'left pending'))
                self.assertEqual((right.status, right.reason),
                                 (ServiceStatus.PARTIAL_SUCCESS, 'right diagnostic'))
                if nested:
                    self.assertTrue(left.info.touch_available is False)
                    self.assertEqual(left.info.runtime_id, 11)
                    self.assertEqual(right.info.runtime_id, 11)
                else:
                    self.assertIsNone(left.info)
                    self.assertIsNone(right.info)

    def test_nested_unified_info_preserves_every_frozen_field(self):
        from copy import deepcopy
        from lowlevel.ee.EndEffectorInfo import EndEffectorInfoT, EndEffectorInfo
        from locomotion_aorta.end_effector_types import EndEffectorInfo as InfoDTO
        from locomotion_aorta._end_effector_codec import decode_info, validate_info
        expected = dict(
            protocol_version=1, runtime_id=2**64-1, layout_id=65538, side=1,
            device_model=2, firmware='固件-μ', serial='serial-Ω',
            identity_valid_mask=7, touch_available=False,
            ts_descriptor_acquired_us=2**63+1, ts_state_pub_us=2**64-1,
            time_valid_mask=3,
        )
        vectors = dict(
            axis_names=('thumb_pitch', '食指', 'middle', 'ring'),
            position_domain=(1, 1, 1, 2),
            torque_domain=(1, 2, 3, 0),
            supported_modes=(0x2, 0x6, 0x2, 0x2),
            supported_command_fields=(0x5F, 0x5F, 0x5F, 0x5F),
            supported_state_fields=(0xBB, 0xBB, 0xBB, 0xBB),
            valid_limits=(0x1F, 0x0F, 0x03, 0x01),
            pos_min=(-.5, -0., -.25, .125),
            pos_max=(.5, .625, 1., 1.125),
            vel_min=(0., -0., -2., 0.),
            vel_max=(2., 4., 6., 8.),
            vel_limit_max=(0., 1., math.inf, 10.),
            tau_min=(-1., -0., -4., 0.),
            tau_max=(1., 2., 4., 8.),
            current_min=(0., -0., -.5, 0.),
            current_max=(1., 2., .5, 8.),
            is_mock=(False, True, False, True),
        )
        obj = EndEffectorInfoT()
        obj.protocolVersion = expected['protocol_version']
        obj.runtimeId = expected['runtime_id']
        obj.layoutId = expected['layout_id']
        obj.side = expected['side']
        obj.deviceModel = expected['device_model']
        obj.firmware = expected['firmware']
        obj.serial = expected['serial']
        obj.identityValidMask = expected['identity_valid_mask']
        obj.touchAvailable = expected['touch_available']
        obj.tsDescriptorAcquiredUs = expected['ts_descriptor_acquired_us']
        obj.tsStatePubUs = expected['ts_state_pub_us']
        obj.timeValidMask = expected['time_valid_mask']
        for name, value in vectors.items():
            parts = name.split('_')
            setattr(obj, parts[0] + ''.join(p.capitalize() for p in parts[1:]), list(value))
        info = module('RobotInfo').RobotInfoT(protocolVersion=2, tsInfoUs=1234, endEffectors=[
            module('EndEffectorCapability').EndEffectorCapabilityT(
                side=1, isMock=False, info=obj, ready=True,
                status=ServiceStatus.SUCCESS, reason='unified fidelity'),
            module('EndEffectorCapability').EndEffectorCapabilityT(
                side=2, isMock=True, ready=False, status=ServiceStatus.UNAVAILABLE,
                reason='no endpoint')])
        builder = flatbuffers.Builder(1)
        builder.Finish(info.Pack(builder))
        wire = module('RobotInfo').RobotInfo.GetRootAs(builder.Output())
        nested = codec.decode_robot_info(wire).end_effectors[0].info
        standalone_builder = flatbuffers.Builder(1)
        standalone_builder.Finish(obj.Pack(standalone_builder))
        standalone = decode_info(EndEffectorInfo.GetRootAs(standalone_builder.Output()))

        for result in (nested, standalone):
            self.assertIs(type(result), InfoDTO)
            for name, value in expected.items():
                with self.subTest(path='nested' if result is nested else 'standalone', field=name):
                    self.assertEqual(getattr(result, name), value)
            for name, value in vectors.items():
                with self.subTest(path='nested' if result is nested else 'standalone', field=name):
                    actual = getattr(result, name)
                    if name == 'axis_names':
                        self.assertEqual(actual, value)
                    else:
                        self.assertEqual(tuple(struct.pack('<f', item) if isinstance(item, float) else item
                                               for item in actual),
                                         tuple(struct.pack('<f', item) if isinstance(item, float) else item
                                               for item in value))
        raw = deepcopy(obj)
        raw.posMin[2] = math.nan
        builder = flatbuffers.Builder(1); builder.Finish(raw.Pack(builder))
        decoded_raw = decode_info(EndEffectorInfo.GetRootAs(builder.Output()))
        self.assertTrue(math.isnan(decoded_raw.pos_min[2]))
        with self.assertRaises(ValueError):
            validate_info(decoded_raw)

    def test_robot_info_preserves_real_body_and_failed_end_effector_in_partial_reply(self):
        # The actual final public object API packs regular tables/vector structs.
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse
        from lowlevel.RobotInfo import RobotInfoT
        from lowlevel.BodyInfo import BodyInfoT
        from lowlevel.BodyJointCapability import BodyJointCapabilityT
        from lowlevel.BodyMotorIdentity import BodyMotorIdentityT
        from lowlevel.EndEffectorCapability import EndEffectorCapabilityT
        body = BodyInfoT(rosterVersion=8, slotCount=31, jointNames=['j'+str(i) for i in range(31)],
                         joints=[BodyJointCapabilityT(bodyIndex=i, slotRole=BodySlotRole.ACTIVE,
                                                      isMock=i == 14, supportedFields=31 if i != 14 else 0,
                                                      validLimits=1 if i != 14 else 0, qMin=-.5, qMax=1.5) for i in range(31)],
                         motorIdentities=[BodyMotorIdentityT(bus='can5', nodeId=i+1, present=i != 14) for i in range(31)])
        info = RobotInfoT(protocolVersion=2, tsInfoUs=1234, body=body, endEffectors=[
            EndEffectorCapabilityT(side=1, isMock=True, ready=False,
                                   status=ServiceStatus.UNAVAILABLE, reason='no endpoint'),
            EndEffectorCapabilityT(side=2, isMock=False, ready=False,
                                   status=ServiceStatus.INTERNAL_ERROR, reason='initialization failed')])
        reply = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS, runtimeId=11, info=info, errorMessage='hands unavailable')
        b = flatbuffers.Builder(1); b.Finish(reply.Pack(b))
        result = codec.decode_robot_info_response(GetRobotInfoResponse.GetRootAs(b.Output()))
        self.assertEqual(result.status, ServiceStatus.PARTIAL_SUCCESS)
        self.assertEqual(result.info.body.joint_names, tuple('j'+str(i) for i in range(31)))
        self.assertEqual(result.info.body.joints[0].q_max, 1.5)
        self.assertFalse(result.info.body.motor_identities[14].present)
        self.assertEqual(result.info.end_effectors[0].reason, 'no endpoint')
        self.assertIsNone(result.info.end_effectors[0].info)
        failed = result.info.end_effectors[1]
        self.assertFalse(failed.is_mock)
        self.assertFalse(failed.ready)
        self.assertEqual((failed.status, failed.reason),
                         (ServiceStatus.INTERNAL_ERROR, 'initialization failed'))

    def test_robot_info2_endpoints_use_six_fields_with_optional_info(self):
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse
        from lowlevel.RobotInfo import RobotInfoT
        from lowlevel.EndEffectorCapability import EndEffectorCapabilityT, EndEffectorCapability
        self.assertFalse(hasattr(EndEffectorCapability, 'GetInfoService'))
        self.assertFalse(hasattr(EndEffectorCapabilityT(), 'getInfoService'))
        # Final unified capability DTO is the six-field contract.
        self.assertEqual([f.name for f in dataclasses.fields(codec.EndEffectorCapability)],
                         ['side', 'is_mock', 'info', 'ready', 'status', 'reason'])
        left_info = self._synthetic_endpoint_info(side=1, runtime=11, n=1, mock=False)
        info = RobotInfoT(protocolVersion=2, tsInfoUs=1234, endEffectors=[
            EndEffectorCapabilityT(side=1, isMock=False, info=left_info,
                                  ready=True, status=ServiceStatus.SUCCESS,
                                  reason='available'),
            EndEffectorCapabilityT(side=2, isMock=True, ready=False,
                                  status=ServiceStatus.PARTIAL_SUCCESS, reason='diagnostic'),
        ])
        response = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS, runtimeId=11, info=info,
                                         errorMessage='preserve aggregate diagnostic')
        b = flatbuffers.Builder(1); b.Finish(response.Pack(b))
        data = bytes(b.Output())
        wire = GetRobotInfoResponse.GetRootAs(data)
        left, right = wire.Info().EndEffectors(0), wire.Info().EndEffectors(1)
        # Actual generated bytes prove the six-field slot layout (vtable size
        # 4 + 6*2), rather than accepting an older wheel's old positions.
        for view in (left, right):
            vtable = view._tab.Pos - struct.unpack_from('<i', data, view._tab.Pos)[0]
            self.assertEqual(struct.unpack_from('<H', data, vtable)[0], 4+6*2)
        result = codec.decode_robot_info_response(wire)
        self.assertEqual(result.status, ServiceStatus.PARTIAL_SUCCESS)
        self.assertEqual(result.error_message, 'preserve aggregate diagnostic')
        left_dto, right_dto = result.info.end_effectors
        self.assertEqual((left_dto.side, left_dto.is_mock, left_dto.ready, left_dto.status, left_dto.reason),
                         (1, False, True, ServiceStatus.SUCCESS, 'available'))
        self.assertEqual(left_dto.info.axis_names, ('axis0',))
        self.assertEqual(left_dto.info.runtime_id, 11)
        self.assertEqual((right_dto.side, right_dto.is_mock, right_dto.ready, right_dto.status, right_dto.reason),
                         (2, True, False, ServiceStatus.PARTIAL_SUCCESS, 'diagnostic'))
        self.assertIsNone(right_dto.info)
        self.assertFalse(hasattr(left_dto, 'get_info_service'))

    def test_robot_info_rejects_wrong_version_and_inconsistent_endpoints(self):
        # B-52A-INFO-1 negatives at the real decode entry.
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse
        from lowlevel.RobotInfo import RobotInfoT
        from lowlevel.EndEffectorCapability import EndEffectorCapabilityT
        def reply_for(info_version=2, endpoint_is_mock=True, info_mock=True,
                      outer_runtime=11, inner_runtime=11, n=1, side=1, bad_shape=False):
            obj = self._synthetic_endpoint_info(side=side, runtime=inner_runtime, n=n, mock=info_mock) \
                if info_mock is not None else None
            if obj is not None and bad_shape:
                obj.isMock = []
            info = RobotInfoT(protocolVersion=info_version, tsInfoUs=1, endEffectors=[
                EndEffectorCapabilityT(side=1, isMock=endpoint_is_mock, info=obj,
                                      ready=False, status=ServiceStatus.UNAVAILABLE, reason='x'),
                EndEffectorCapabilityT(side=2, isMock=True, ready=False,
                                      status=ServiceStatus.UNAVAILABLE, reason='y'),
            ])
            reply = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS, runtimeId=outer_runtime,
                                          info=info, errorMessage='n/a')
            b = flatbuffers.Builder(1); b.Finish(reply.Pack(b))
            return GetRobotInfoResponse.GetRootAs(b.Output())
        cases = (({'info_version': 1}, 'RobotInfo protocol_version'),
                 ({'info_version': 3}, 'RobotInfo protocol_version'),
                 ({'endpoint_is_mock': False, 'info_mock': True}, 'is_mock conflicts'),
                 ({'endpoint_is_mock': True, 'info_mock': False}, 'is_mock conflicts'),
                 ({'inner_runtime': 99}, 'runtime_id differs'),
                 ({'outer_runtime': 0}, 'requires nonzero runtime_id'),
                 ({'n': 0}, 'axis_names must be nonempty'),
                 ({'bad_shape': True}, 'is_mock.*fixed length'),
                 ({'side': 2}, 'side differs'))
        for kwargs, message in cases:
            with self.subTest(**kwargs):
                with self.assertRaisesRegex(ValueError, message):
                    codec.decode_robot_info_response(reply_for(**kwargs))
        # Valid mock agreement decodes.
        result = codec.decode_robot_info_response(reply_for())
        self.assertTrue(result.info.end_effectors[0].is_mock)
        # A diagnostic None reply stays distinct from a BodyOnly description.
        b = flatbuffers.Builder(1)
        reply = GetRobotInfoResponseT(status=ServiceStatus.SUCCESS, runtimeId=11, info=None,
                                      errorMessage='')
        b.Finish(reply.Pack(b))
        no_info = codec.decode_robot_info_response(GetRobotInfoResponse.GetRootAs(b.Output()))
        self.assertIsNone(no_info.info)
        from lowlevel.BodyInfo import BodyInfoT
        from lowlevel.BodyJointCapability import BodyJointCapabilityT
        from lowlevel.BodyMotorIdentity import BodyMotorIdentityT
        body = BodyInfoT(slotCount=31, jointNames=['j'+str(i) for i in range(31)],
                         joints=[BodyJointCapabilityT(bodyIndex=i) for i in range(31)],
                         motorIdentities=[BodyMotorIdentityT() for _ in range(31)])
        info = RobotInfoT(protocolVersion=2, body=body, endEffectors=[
            EndEffectorCapabilityT(side=side, isMock=True, ready=False,
                                   status=ServiceStatus.UNAVAILABLE, reason='no endpoint')
            for side in (1, 2)])
        b = flatbuffers.Builder(1)
        b.Finish(GetRobotInfoResponseT(status=ServiceStatus.SUCCESS, runtimeId=11, info=info).Pack(b))
        body_only = codec.decode_robot_info_response(GetRobotInfoResponse.GetRootAs(b.Output()))
        self.assertEqual(body_only.info.body.slot_count, 31)
        self.assertTrue(all(slot.info is None and not slot.ready for slot in body_only.info.end_effectors))
        b = flatbuffers.Builder(1)
        b.Finish(GetRobotInfoResponseT(status=ServiceStatus.UNAVAILABLE, runtimeId=0,
                                       info=info, errorMessage='startup failed').Pack(b))
        unbound = codec.decode_robot_info_response(GetRobotInfoResponse.GetRootAs(b.Output()))
        self.assertEqual((unbound.runtime_id, unbound.status, unbound.error_message),
                         (0, ServiceStatus.UNAVAILABLE, 'startup failed'))
        self.assertEqual(unbound.info.body.slot_count, 31)

    def test_applied_correlation_keeps_zero_defaults_and_full_uint64_keys(self):
        m = module('BodyState')
        for session, sequence, stamp in ((0, 0, 0), (2**64-1, 2**64-2, 99)):
            obj = m.BodyStateT.InitFromObj(make_state()[1])
            obj.appliedSessionId, obj.appliedSequence, obj.tsFirstOutputUs = session, sequence, stamp
            builder = flatbuffers.Builder(1)
            builder.Finish(obj.Pack(builder))
            state = codec.decode_body_state(m.BodyState.GetRootAs(builder.Output()))
            self.assertEqual((state.applied_session_id, state.applied_sequence, state.ts_first_output_us),
                             (session, sequence, stamp))
            self.assertFalse(hasattr(state, 'applied_cmd'))

    def test_state3_association_preserves_all_sources_zero_keys_and_full_width_ref(self):
        from locomotion_aorta.body_types import BODY_CMD_PROTOCOL_VERSION, BODY_STATE_PROTOCOL_VERSION
        self.assertEqual((BODY_CMD_PROTOCOL_VERSION, BODY_STATE_PROTOCOL_VERSION), (3, 3))
        m = module('BodyState')
        for source in (0, 1, 2, 3, 4, 5, 6, 255):
            for key, valid in (((0, 0), True), ((2**64-1, 2**63+3), True), ((99, 101), False)):
                with self.subTest(source=source, key=key, valid=valid):
                    obj = m.BodyStateT.InitFromObj(make_state()[1])
                    obj.stateId = 2**64-2
                    obj.runtimeId = 2**64-1
                    obj.tsControlUs, obj.appliedKeyValid, obj.associationSourceType = 2**64-3, valid, source
                    obj.appliedSessionId, obj.appliedSequence, obj.tsFirstOutputUs = *key, 0
                    builder = flatbuffers.Builder(1)
                    builder.Finish(obj.Pack(builder))
                    state = codec.decode_body_state(m.BodyState.GetRootAs(builder.Output()))
                    self.assertEqual(state.runtime_id, 2**64-1)
                    self.assertEqual((state.ts_control_us, state.applied_key_valid, state.association_source_type),
                                     (2**64-3, valid, source))
                    self.assertEqual((state.applied_session_id, state.applied_sequence, state.ts_first_output_us),
                                     (*key, 0))
                    self.assertEqual(state.state_id, 2**64-2)
        state = codec.decode_body_state(make_state()[1])
        # Ordinary positives carry the fixture's explicit nonzero runtime.
        self.assertEqual(state.runtime_id, BODY_RUNTIME_ID)
        self.assertEqual((state.ts_control_us, state.applied_key_valid, state.association_source_type), (0, False, 0))

    def test_state3_missing_runtime_field_is_zero_and_rejected(self):
        # Missing runtime id is a real negative: the scalar decodes to 0 and
        # both decoders reject it; the ordinary positive above uses a nonzero R.
        builder = flatbuffers.Builder(1)
        m = module('BodyState')
        joints = _minimal_joints_vector(builder)
        m.BodyStateStart(builder)
        m.BodyStateAddProtocolVersion(builder, 3)
        m.BodyStateAddStateId(builder, 7)
        m.BodyStateAddJoints(builder, joints)
        builder.Finish(m.BodyStateEnd(builder))
        payload = bytes(builder.Output())
        with self.assertRaisesRegex(ValueError, 'nonzero runtime_id'):
            codec._decode_body_state_python(m.BodyState.GetRootAs(payload))
        from locomotion_aorta._native_body_codec import decode_body_state_native
        with self.assertRaisesRegex(ValueError, 'nonzero runtime_id'):
            decode_body_state_native(payload)

    def test_state3_rejects_present_zero_runtime(self):
        obj = module('BodyState').BodyStateT.InitFromObj(make_state()[1])
        obj.runtimeId = 0
        builder = flatbuffers.Builder(1)
        builder.Finish(obj.Pack(builder))
        with self.assertRaisesRegex(ValueError, 'nonzero runtime_id'):
            codec.decode_body_state(module('BodyState').BodyState.GetRootAs(builder.Output()))

    def test_decoder_preserves_real_error_even_with_nonfinite_feedback(self):
        obj = module('BodyState').BodyStateT.InitFromObj(make_state()[1])
        obj.joints[1].q = math.nan
        obj.joints[1].errorCode = 3  # Real over-current takes priority at the producer.
        obj.joints[2].tsIngestedUs = 0
        obj.joints[2].errorCode = 8
        builder = flatbuffers.Builder(1)
        builder.Finish(obj.Pack(builder))
        state = codec.decode_body_state(module('BodyState').BodyState.GetRootAs(builder.Output()))
        self.assertTrue(math.isnan(state.joints[1].q))
        self.assertEqual((state.joints[1].error_code, state.joints[2].error_code), (3,8))

    def test_state_id_is_copied_to_command_without_uint64_truncation(self):
        for state_id in (2**32+7, 2**63+9, 2**64-1):
            obj = module('BodyState').BodyStateT.InitFromObj(make_state()[1])
            obj.stateId = state_id
            builder = flatbuffers.Builder(1)
            builder.Finish(obj.Pack(builder))
            state = codec.decode_body_state(module('BodyState').BodyState.GetRootAs(builder.Output()))
            command = dataclasses.replace(make_command(), input_state_id=state.state_id)
            decoded = module('BodyCmd').BodyCmd.GetRootAs(encode(command))
            self.assertEqual((state.state_id, decoded.InputStateId()), (state_id, state_id))
            self.assertFalse(hasattr(decoded, 'CmdId'))

    def test_typed_rpc_uses_real_request_response_and_keeps_failed_status(self):
        from unittest.mock import patch
        import locomotion_aorta.transport as transport
        from aorta.services.humanoid_locomotion import GetRobotInfoRequest as request_module
        from aorta.services.humanoid_locomotion import GetRobotInfoResponse as reply_module
        class Client:
            matches = []
            def raw(self): return self
            def wait_for_matching(self, timeout): self.matches.append(timeout); return True
            def call(self, fill):
                b = flatbuffers.Builder(1); b.Finish(fill(b,0))
                request_module.GetRobotInfoRequest.GetRootAs(b.Output())
                b = flatbuffers.Builder(1)
                response = reply_module.GetRobotInfoResponseT(status=ServiceStatus.UNAVAILABLE,
                    runtimeId=0, info=None, errorMessage='startup unavailable')
                b.Finish(response.Pack(b))
                view = reply_module.GetRobotInfoResponse.GetRootAs(b.Output())
                return SimpleNamespace(decode=lambda: view)
        client = Client()
        class Node:
            calls = []
            def __init__(self, *a): pass
            def group(self): return 'default'
            def create_publisher_typed(self, *a, **kw): return object()
            def create_client_typed(self, *a, **kw): self.calls.append((a,kw)); return client
            def close(self): pass
        qos = SimpleNamespace(realtime_control=lambda: None)
        fake_aorta = SimpleNamespace(Node=Node, QoS=qos, ClientOptions=lambda **kw: kw)
        bindings = SimpleNamespace(aorta=fake_aorta, control_request_schema_meta=object(), body_cmd_schema_meta=object(),
                                   get_robot_info_request=request_module, get_robot_info_response=reply_module,
                                   get_robot_info_schema_meta=importlib.import_module('get_robot_info_schema_meta'))
        with patch.object(transport,'_load_aorta_bindings',return_value=bindings), \
             patch('locomotion_aorta._native_body_codec.NativeBodyPublisher'):
            t = transport.create_aorta_transport(node_name='rpc-test',group='default')
        try:
            for _ in range(2):
                reply = t.get_robot_info(.5)
                self.assertEqual(reply.status, ServiceStatus.UNAVAILABLE)
                self.assertIsNone(reply.info)
                self.assertEqual((reply.runtime_id, reply.error_message), (0, 'startup unavailable'))
            self.assertEqual(len(t.node.calls),1)
            self.assertEqual(t.node.calls[0][0][0],transport.GET_ROBOT_INFO_SERVICE)
            self.assertEqual(client.matches,[500,500])
            self.assertEqual(set(t.publishers),{transport.BODY_CMD_TOPIC, transport.CONTROL_REQUEST_TOPIC})
            from fake_transport import FakeTransport
            from locomotion_aorta.controller import ExternalController
            fake = FakeTransport()
            fake.get_robot_info = lambda timeout: reply
            controller = ExternalController(client_id=7, _transport=fake)
            try:
                self.assertEqual(controller.get_robot_info(.1), reply)
                self.assertIsNone(controller._end_effector_runtime_id)
                self.assertFalse(any(topic == transport.CONTROL_REQUEST_TOPIC for topic, _ in fake.published))
            finally:
                controller.close()
        finally: t.close()

    def test_corrupt_parent_vtable_and_vector_offsets_are_rejected(self):
        original, view = make_state()
        root = view._tab.Pos
        vtable = root - struct.unpack_from('<i', original, root)[0]
        cases = ((vtable, 3), (vtable + 2, 4),
                 (vtable + codec._STATE_JOINTS_SLOT, 1),
                 (vtable + codec._STATE_JOINTS_SLOT, 65535))
        for at, value in cases:
            data = bytearray(original); struct.pack_into('<H',data,at,value)
            with self.subTest(at=at,value=value), self.assertRaises((ValueError,struct.error)):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(data))
        broken = bytearray(original);struct.pack_into('<i',broken,root,2**30)
        with self.assertRaises(ValueError):
            codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(broken))

    def test_corrupt_vector_length_offset_and_alignment_are_rejected(self):
        original, view = make_state()
        field = view._tab.Pos + view._tab.Offset(codec._STATE_JOINTS_SLOT)
        vector = field + struct.unpack_from('<I', original, field)[0]
        for count in (0, 30, 32, 42, 44, 0xffffffff):
            data = bytearray(original); struct.pack_into('<I', data, vector, count)
            with self.subTest(count=count), self.assertRaises(ValueError):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(data))
        for offset in (0, 1, 0xffffffff):
            data = bytearray(original); struct.pack_into('<I', data, field, offset)
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(data))

class RobotInfoRuntimeR3Test(unittest.TestCase):
    def test_partial_info_reply_decodes_same_response_runtime(self):
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse

        info = module('RobotInfo').RobotInfoT(protocolVersion=2, tsInfoUs=987, endEffectors=[
            module('EndEffectorCapability').EndEffectorCapabilityT(
                side=side, isMock=True, ready=False, status=ServiceStatus.UNAVAILABLE,
                reason='no endpoint') for side in (1, 2)])
        response = GetRobotInfoResponseT(
            status=ServiceStatus.PARTIAL_SUCCESS, info=info,
            errorMessage='right unavailable', runtimeId=11)
        builder = flatbuffers.Builder(1)
        builder.Finish(response.Pack(builder))
        decoded = codec.decode_robot_info_response(GetRobotInfoResponse.GetRootAs(builder.Output()))
        self.assertEqual(decoded.status, ServiceStatus.PARTIAL_SUCCESS)
        self.assertEqual(decoded.error_message, 'right unavailable')
        self.assertEqual(decoded.runtime_id, 11)
        self.assertEqual(decoded.info.ts_info_us, 987)
