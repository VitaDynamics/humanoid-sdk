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
    BodySlotRole, EndEffectorKind, ServiceStatus,
)
from locomotion_aorta import _body_codec as codec
from locomotion_aorta.transport import _fill_body_cmd


def module(name):
    return importlib.import_module('lowlevel.' + name)


def make_command(seed=0):
    return BodyCmd(17, tuple(BodyJointCmd(i + seed + .125, -i - .25, i * .5, i + 1., .75)
                            for i in range(31)), action=BodyCmdAction.EXTERNAL,
                  session_id=42, sequence=7, input_state_id=123, input_state_ts_real_us=456,
                  ts_client_created_us=789, ts_client_send_us=901)


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
        m.BodyCmdAddProtocolVersion(builder, command.protocol_version)
        m.BodyCmdAddSessionId(builder, command.session_id)
        m.BodyCmdAddSequence(builder, command.sequence)
        m.BodyCmdAddCmdId(builder, command.cmd_id)
        m.BodyCmdAddInputStateId(builder, command.input_state_id)
        m.BodyCmdAddInputStateTsRealUs(builder, command.input_state_ts_real_us)
        m.BodyCmdAddTsClientCreatedUs(builder, command.ts_client_created_us)
        m.BodyCmdAddTsClientSendUs(builder, command.ts_client_send_us)
        builder.ForceDefaults(True)
        m.BodyCmdAddAction(builder, command.action)
        builder.ForceDefaults(False)
        if command.root_quat is not None:
            m.BodyCmdAddRootQuat(builder, module('RootQuatWxyz').CreateRootQuatWxyz(builder, *command.root_quat))
        root = m.BodyCmdEnd(builder)
    else:
        root = _fill_body_cmd(SimpleNamespace(body_cmd_module=m), command, builder, 0)
    builder.Finish(root)
    return bytes(builder.Output())


def make_state():
    builder = flatbuffers.Builder(1)
    # Both IMUs retain original clocks/units; vector order intentionally TRUNK,HIP.
    imu_module = module('BodyImuSample')
    imus = []
    for location in (BodyImuLocation.TRUNK, BodyImuLocation.HIP):
        imu_module.BodyImuSampleStart(builder)
        imu_module.BodyImuSampleAddLocation(builder, location)
        imu_module.BodyImuSampleAddState(builder, module('BodyImuState').CreateBodyImuState(
            builder, (1., 0., 0., 0.), (.1, .2, .3), (0., 0., 9.81), (-1., 2., 3.)))
        imu_module.BodyImuSampleAddStatus(builder, BodySampleStatus.OK)
        imu_module.BodyImuSampleAddTsSampledUs(builder, 101)
        imu_module.BodyImuSampleAddTsAcquiredHwUs(builder, 102)
        imu_module.BodyImuSampleAddTsIngestedUs(builder, 103)
        imu_module.BodyImuSampleAddTsAssembledUs(builder, 104)
        imu_module.BodyImuSampleAddDeviceTimestampMs(builder, 876)
        imu_module.BodyImuSampleAddHostTimestampUs(builder, 987654)
        imus.append(imu_module.BodyImuSampleEnd(builder))
    m = module('BodyState')
    m.BodyStateStartImuStatesVector(builder, len(imus))
    for imu in reversed(imus): builder.PrependUOffsetTRelative(imu)
    imus = builder.EndVector()
    cols = [tuple(range(31)),
            tuple(BodySampleStatus.UNCONFIGURED if i == 14 else BodySampleStatus.OK for i in range(31)),
            tuple(range(31)), tuple(1000 + i for i in range(31)), tuple(2000 + i for i in range(31)),
            tuple(math.nan if i == 14 else i + .125 for i in range(31)),
            (-.25,) * 31, (-.75,) * 31, tuple(-7 if i == 30 else i for i in range(31))]
    m.BodyStateStartJointsVector(builder, 31)
    for values in reversed(tuple(zip(*cols))):
        module('BodyJointState').CreateBodyJointState(builder, *values)
    joints = builder.EndVector()
    m.BodyStateStart(builder)
    m.BodyStateAddProtocolVersion(builder, 2)
    m.BodyStateAddStateId(builder, 91)
    m.BodyStateAddTsStateRealUs(builder, 1_234_567)
    m.BodyStateAddTsStatePubUs(builder, 1_234_890)
    m.BodyStateAddImuStates(builder, imus)
    m.BodyStateAddAttitudeValid(builder, True)
    m.BodyStateAddJoints(builder, joints)
    builder.Finish(m.BodyStateEnd(builder))
    data = bytes(builder.Output())
    return data, m.BodyState.GetRootAs(data)


class BodyCodecTest(unittest.TestCase):
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
                        self.assertNotEqual(view._tab.Offset(24), 0)  # action id10 is explicitly present
                        self.assertEqual(view.JointsLength(), 31)
                        self.assertEqual(view.Joints(30).Q(), command.joints[30].q)

    def test_no_vector_or_wrong_width_is_accepted_by_bulk(self):
        for count in (0, 30, 32, 42, 44):
            with self.subTest(count=count), self.assertRaises(ValueError):
                codec.create_body_cmd(flatbuffers.Builder(1), (BodyJointCmd(),) * count)

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
        self.assertEqual(state.joints[14].status, BodySampleStatus.UNCONFIGURED)
        self.assertEqual(state.joints[0].error_code, 0)
        self.assertEqual(state.joints[30].error_code, -7)
        self.assertEqual(state.joints[0].tau_est, -.75)
        self.assertIsNone(state.applied_cmd)
        self.assertEqual([s.location for s in state.imu_states], [BodyImuLocation.TRUNK, BodyImuLocation.HIP])
        self.assertEqual(state.imu_states[1].state.rpy, (-1., 2., 3.))
        self.assertEqual(state.imu_states[1].device_timestamp_ms, 876)
        self.assertEqual(state.imu_states[1].host_timestamp_us, 987654)
        sample = state.imu_states[1]
        self.assertEqual((sample.ts_sampled_us, sample.ts_acquired_hw_us,
                          sample.ts_ingested_us, sample.ts_assembled_us), (101, 102, 103, 104))

    def test_v1_is_rejected_before_any_fixed_stride_or_imu_decode(self):
        from types import SimpleNamespace
        # Only protocol_version is safe to read from the incompatible v1 frame.
        for version in (0, 1, 3):
            with self.subTest(version=version), self.assertRaisesRegex(ValueError, 'protocol_version'):
                codec.decode_body_state(SimpleNamespace(ProtocolVersion=lambda: version))

    def test_joint_error_enum_and_removed_dto_fields_follow_public_bindings(self):
        from locomotion_aorta import JointError, BodyJointState, BodyImuSample
        self.assertEqual((JointError.NONE, JointError.TIMEOUT, JointError.UNKNOWN), (0, 20, 255))
        self.assertEqual([f.name for f in dataclasses.fields(BodyJointState)],
                         ['body_index', 'status', 'ts_acquired_hw_us', 'ts_ingested_us',
                          'ts_assembled_us', 'q', 'dq', 'tau_est', 'error_code'])
        for cls in (BodyJointState, BodyImuSample):
            names = {f.name for f in dataclasses.fields(cls)}
            self.assertFalse(names & {'populated', 'is_mock', 'valid_fields', 'ddq', 'temperature_deg_c', 'lost'})
        # Preserve unknown numeric errors; do not add an enum-range gate.
        sample = BodyJointState(0, BodySampleStatus.OK, error_code=123)
        self.assertEqual(sample.error_code, 123)

    def test_truncated_vector_payload_is_rejected(self):
        data, view = make_state()
        pos = view.Joints(0)._tab.Pos
        for missing in (1, 20, 48, 1488):
            short = data[:pos + 1488 - missing]
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
             [0,2,8,16,24,32,36,40,44]),
            ('BodyJointCmd', ['Q','Dq','Tau','Kp','Kd'], [0,4,8,12,16]),
            ('BodyState', ['JointsLength'], [22]),
            ('BodyImuSample', ['DeviceTimestampMs', 'HostTimestampUs'], [20,22]),
            ('BodyCmd', ['JointsLength','Action','RootQuat'], [22,24,26]),
        ):
            view = getattr(module(name), name)(); probe = Probe(); view._tab = probe
            for getter in getters: getattr(view, getter)()
            self.assertEqual(probe.positions, expected)

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
                                                      populated=i != 14, supportedFields=31 if i != 14 else 0,
                                                      validLimits=1 if i != 14 else 0, qMin=-.5, qMax=1.5) for i in range(31)],
                         motorIdentities=[BodyMotorIdentityT(bus='can5', nodeId=i+1, present=i != 14) for i in range(31)])
        info = RobotInfoT(protocolVersion=1, tsInfoUs=1234, body=body,
                         endEffectors=[EndEffectorCapabilityT(side=side, kind=EndEffectorKind.HAND,
                                                              configured=True, ready=False,
                                                              status=ServiceStatus.UNAVAILABLE, reason='modbus unavailable') for side in (1,2)])
        reply = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS, info=info, errorMessage='hands unavailable')
        b = flatbuffers.Builder(1); b.Finish(reply.Pack(b))
        result = codec.decode_robot_info_response(GetRobotInfoResponse.GetRootAs(b.Output()))
        self.assertEqual(result.status, ServiceStatus.PARTIAL_SUCCESS)
        self.assertEqual(result.info.body.joint_names, tuple('j'+str(i) for i in range(31)))
        self.assertEqual(result.info.body.joints[0].q_max, 1.5)
        self.assertFalse(result.info.body.motor_identities[14].present)
        self.assertEqual(result.info.end_effectors[0].reason, 'modbus unavailable')
        self.assertIsNone(result.info.end_effectors[0].hand)

    def test_r1_nested_end_effectors_use_nine_fields_without_info_endpoint(self):
        from aorta.services.humanoid_locomotion.GetRobotInfoResponse import GetRobotInfoResponseT, GetRobotInfoResponse
        from lowlevel.RobotInfo import RobotInfoT
        from lowlevel.EndEffectorCapability import EndEffectorCapabilityT, EndEffectorCapability
        from lowlevel.HandInfo import HandInfoT
        from lowlevel.GripperInfo import GripperInfoT
        self.assertFalse(hasattr(EndEffectorCapability, 'GetInfoService'))
        self.assertFalse(hasattr(EndEffectorCapabilityT(), 'getInfoService'))
        self.assertEqual([f.name for f in dataclasses.fields(codec.EndEffectorCapability)],
                         ['side', 'kind', 'configured', 'is_mock', 'hand', 'gripper',
                          'ready', 'status', 'reason'])
        hand = HandInfoT(side=1, dof=6, qMin=[-.5]*6, qMax=[1.25]*6,
                         dqMax=[2.0]*6)
        gripper = GripperInfoT(side=2, openingMinM=0., openingMaxM=.0625,
                              maxOpeningRateMS=.015625, holdTorqueNm=-.25)
        info = RobotInfoT(protocolVersion=1, endEffectors=[
            EndEffectorCapabilityT(side=1, kind=EndEffectorKind.HAND, configured=True,
                                  hand=hand, ready=True, status=ServiceStatus.SUCCESS,
                                  reason='hand available'),
            EndEffectorCapabilityT(side=2, kind=EndEffectorKind.GRIPPER, configured=True,
                                  gripper=gripper, ready=False,
                                  status=ServiceStatus.PARTIAL_SUCCESS, reason='gripper diagnostic'),
        ])
        response = GetRobotInfoResponseT(status=ServiceStatus.PARTIAL_SUCCESS, info=info,
                                         errorMessage='preserve aggregate diagnostic')
        b = flatbuffers.Builder(1); b.Finish(response.Pack(b))
        data = bytes(b.Output())
        wire = GetRobotInfoResponse.GetRootAs(data)
        left, right = wire.Info().EndEffectors(0), wire.Info().EndEffectors(1)
        # Actual generated bytes prove the shifted slots, rather than accepting
        # an older same-version wheel with the old accessor/field positions.
        for view in (left, right):
            vtable = view._tab.Pos - struct.unpack_from('<i', data, view._tab.Pos)[0]
            self.assertEqual(struct.unpack_from('<H', data, vtable)[0], 4+9*2)
        self.assertNotEqual(left._tab.Offset(12), 0)   # hand id4
        self.assertNotEqual(right._tab.Offset(14), 0)  # gripper id5
        for offset in (16, 18, 20):                   # ready/status/reason ids6/7/8
            self.assertNotEqual(left._tab.Offset(offset), 0)
        result = codec.decode_robot_info_response(wire)
        self.assertEqual(result.status, ServiceStatus.PARTIAL_SUCCESS)
        self.assertEqual(result.error_message, 'preserve aggregate diagnostic')
        hand_dto, grip_dto = result.info.end_effectors
        self.assertEqual((hand_dto.side, hand_dto.ready, hand_dto.status, hand_dto.reason),
                         (1, True, ServiceStatus.SUCCESS, 'hand available'))
        self.assertEqual(hand_dto.hand.q_max, (1.25,)*6)
        self.assertIsNone(hand_dto.gripper)
        self.assertEqual((grip_dto.side, grip_dto.ready, grip_dto.status, grip_dto.reason),
                         (2, False, ServiceStatus.PARTIAL_SUCCESS, 'gripper diagnostic'))
        self.assertEqual(grip_dto.gripper.hold_torque_nm, -.25)
        self.assertIsNone(grip_dto.hand)
        self.assertFalse(hasattr(hand_dto, 'get_info_service'))


    def test_applied_echo_is_optional_and_preserves_generated_unknown_default(self):
        m, applied = module('BodyState'), module('BodyAppliedCmd')
        builder = flatbuffers.Builder(1)
        applied.BodyAppliedCmdStartJointsVector(builder, 31)
        for _ in range(31):
            module('BodyJointCmd').CreateBodyJointCmd(builder, .5,.5,.5,.5,.5)
        joints = builder.EndVector()
        applied.BodyAppliedCmdStart(builder)
        applied.BodyAppliedCmdAddSessionId(builder, 77)
        applied.BodyAppliedCmdAddSequence(builder, 88)
        applied.BodyAppliedCmdAddTsFirstOutputUs(builder, 99)
        applied.BodyAppliedCmdAddSourceKeyValid(builder, True)
        applied.BodyAppliedCmdAddJoints(builder, joints)
        echo = applied.BodyAppliedCmdEnd(builder)
        m.BodyStateStart(builder); m.BodyStateAddProtocolVersion(builder, 2); m.BodyStateAddAppliedCmd(builder, echo)
        builder.Finish(m.BodyStateEnd(builder))
        state = codec.decode_body_state(m.BodyState.GetRootAs(builder.Output()))
        self.assertEqual(state.applied_cmd.action, BodyCmdAction.UNKNOWN)
        self.assertEqual((state.applied_cmd.session_id, state.applied_cmd.sequence, state.applied_cmd.ts_first_output_us), (77,88,99))
        self.assertTrue(state.applied_cmd.source_key_valid)
        self.assertEqual(state.applied_cmd.joints[30], BodyJointCmd(.5,.5,.5,.5,.5))

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
                reply_module.GetRobotInfoResponseStart(b)
                reply_module.GetRobotInfoResponseAddStatus(b, ServiceStatus.UNAVAILABLE)
                b.Finish(reply_module.GetRobotInfoResponseEnd(b))
                view = reply_module.GetRobotInfoResponse.GetRootAs(b.Output())
                return SimpleNamespace(decode=lambda: view)
        client = Client()
        class Node:
            calls = []
            def __init__(self, *a): pass
            def create_publisher_typed(self, *a, **kw): return object()
            def create_client_typed(self, *a, **kw): self.calls.append((a,kw)); return client
            def close(self): pass
        qos = SimpleNamespace(realtime_control=lambda: None)
        fake_aorta = SimpleNamespace(Node=Node, QoS=qos, ClientOptions=lambda **kw: kw)
        bindings = SimpleNamespace(aorta=fake_aorta, control_request_schema_meta=object(), body_cmd_schema_meta=object(),
                                   get_robot_info_request=request_module, get_robot_info_response=reply_module,
                                   get_robot_info_schema_meta=importlib.import_module('get_robot_info_schema_meta'))
        with patch.object(transport,'_load_aorta_bindings',return_value=bindings):
            t = transport.create_aorta_transport(node_name='rpc-test',group='default')
        try:
            for _ in range(2):
                reply = t.get_robot_info(.5)
                self.assertEqual(reply.status, ServiceStatus.UNAVAILABLE)
                self.assertIsNone(reply.info)
            self.assertEqual(len(t.node.calls),1)
            self.assertEqual(t.node.calls[0][0][0],transport.GET_ROBOT_INFO_SERVICE)
            self.assertEqual(client.matches,[500,500])
            self.assertEqual(set(t.publishers),{transport.BODY_CMD_TOPIC, transport.CONTROL_REQUEST_TOPIC})
        finally: t.close()

    def test_corrupt_parent_vtable_and_vector_offsets_are_rejected(self):
        original, view = make_state()
        root = view._tab.Pos
        vtable = root - struct.unpack_from('<i', original, root)[0]
        cases = ((vtable, 3), (vtable + 2, 4),
                 (vtable + codec._BODY_JOINTS_SLOT, 1),
                 (vtable + codec._BODY_JOINTS_SLOT, 65535))
        for at, value in cases:
            data = bytearray(original); struct.pack_into('<H',data,at,value)
            with self.subTest(at=at,value=value), self.assertRaises((ValueError,struct.error)):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(data))
        broken = bytearray(original);struct.pack_into('<i',broken,root,2**30)
        with self.assertRaises(ValueError):
            codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(broken))

    def test_corrupt_vector_length_offset_and_alignment_are_rejected(self):
        original, view = make_state()
        field = view._tab.Pos + view._tab.Offset(codec._BODY_JOINTS_SLOT)
        vector = field + struct.unpack_from('<I', original, field)[0]
        for count in (0, 30, 32, 42, 44, 0xffffffff):
            data = bytearray(original); struct.pack_into('<I', data, vector, count)
            with self.subTest(count=count), self.assertRaises(ValueError):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(data))
        for offset in (0, 1, 0xffffffff):
            data = bytearray(original); struct.pack_into('<I', data, field, offset)
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                codec.unpack_body_joints(module('BodyState').BodyState.GetRootAs(data))
