"""Public 0.1.4 actual wire tests; no transport construction or robot connection."""
from dataclasses import replace
import unittest

from locomotion_aorta import (
    BodyCmd, BodyJointCmd, BodyJointState, BodyState, BodyImuSample,
    BodyImuState, JointError, InvalidCommandError,
)
from locomotion_aorta._body_codec import decode_body_state
from locomotion_aorta.transport import _fill_body_cmd
from types import SimpleNamespace
import importlib
from locomotion_aorta.controller import _validate_command
from lowlevel.BodyState import BodyState as WireBodyState, BodyStateT
from lowlevel.BodyJointState import BodyJointState as WireJoint, BodyJointStateT
from lowlevel.BodyImuSample import BodyImuSampleT
from lowlevel.BodyImuState import BodyImuStateT
from lowlevel.BodyCmd import BodyCmd as WireBodyCmd
import flatbuffers


def encode_state(state):
    builder = flatbuffers.Builder(2048)
    builder.Finish(state.Pack(builder))
    return decode_body_state(WireBodyState.GetRootAs(builder.Output()))


class WbcTrackingBindingTest(unittest.TestCase):
    def test_disabled_trunk_without_state_does_not_discard_feedback(self):
        result = encode_state(BodyStateT(protocolVersion=3, joints=[BodyJointStateT() for _ in range(31)],
                              imuStates=[BodyImuSampleT(location=1, status=6)], stateId=2**63))
        self.assertEqual(result.state_id, 2**63)
        self.assertEqual((result.imu_states[0].location, result.imu_states[0].status), (1, 6))
        self.assertIsNone(result.imu_states[0].state)

    def test_real_binding_decodes_31_joints_clocks_error_and_applied(self):
        imus = [BodyImuSampleT(location=location, status=1,
                    tsSampledUs=100+location, tsAcquiredHwUs=101+location,
                    tsIngestedUs=102+location, tsAssembledUs=103+location,
                    state=BodyImuStateT([1.,0.,0.,0.], [1.,2.,3.], [0.,0.,9.8], [0.,0.,0.]))
                for location in (1,0,2)]
        raw = BodyStateT(protocolVersion=3, rosterVersion=1, stateId=2**64-1,
                tsStateRealUs=1234567890123456, tsStatePubUs=1234567890123500,
                tsConsumedUs=1234567890123490, attitudeValid=True, imuStates=imus,
                joints=[BodyJointStateT(tsAcquiredHwUs=10+i, tsIngestedUs=20+i,
                          tsAssembledUs=30+i, q=i/4, dq=-i/2, tauEst=i,
                          errorCode=JointError.TIMEOUT if i==7 else JointError.NONE) for i in range(31)],
                appliedSessionId=2**63, appliedSequence=2**64-1, tsFirstOutputUs=987654)
        result = encode_state(raw)
        self.assertEqual(WireJoint.SizeOf(), 40)
        self.assertEqual((result.state_id, result.applied_session_id, result.applied_sequence),
                         (2**64-1, 2**63, 2**64-1))
        self.assertEqual(result.ts_first_output_us, 987654)
        self.assertEqual(len(result.joints), 31)
        self.assertEqual((result.joints[30].q, result.joints[30].dq), (7.5,-15.))
        self.assertEqual(result.joints[7].error_code, JointError.TIMEOUT)
        self.assertEqual((result.joints[7].ts_acquired_hw_us, result.joints[7].ts_ingested_us,
                          result.joints[7].ts_assembled_us), (17,27,37))
        self.assertEqual([i.location for i in result.imu_states], [1,0,2])
        self.assertEqual(result.imu_states[1].state.quaternion, (1.,0.,0.,0.))

    def test_real_binding_encodes_uint64_keys_and_input_reference(self):
        command = BodyCmd((BodyJointCmd(q=.25),)*31, session_id=2**63, sequence=2**64-1,
                          input_state_id=2**64-1, input_state_ts_real_us=1234567890123456)
        _validate_command(command)
        builder = flatbuffers.Builder(1024)
        builder.Finish(_fill_body_cmd(SimpleNamespace(body_cmd_module=importlib.import_module('lowlevel.BodyCmd')), command, builder, 0))
        view = WireBodyCmd.GetRootAs(builder.Output())
        self.assertEqual((view.SessionId(), view.Sequence(), view.InputStateId()),
                         (2**63, 2**64-1, 2**64-1))
        self.assertEqual(view.InputStateTsRealUs(), command.input_state_ts_real_us)
        self.assertEqual(view.JointsLength(), 31)
        self.assertEqual(view.Joints(30).Q(), .25)
        self.assertFalse(hasattr(view, 'CmdId'))

    def test_invalid_command_rejected_before_transport(self):
        command = BodyCmd((BodyJointCmd(),)*31)
        for bad in (replace(command, joints=command.joints[:30]),
                    replace(command, joints=(BodyJointCmd(q=float('nan')),)*31),
                    replace(command, action=255)):
            with self.subTest(bad=bad), self.assertRaises(InvalidCommandError):
                _validate_command(bad)
        # The generated encoder, rather than a new demo guard, bounds uint64.
        for value in (-1, 2**64):
            with self.subTest(value=value), self.assertRaises((ValueError, TypeError)):
                builder = flatbuffers.Builder(1024)
                _fill_body_cmd(SimpleNamespace(body_cmd_module=importlib.import_module('lowlevel.BodyCmd')),
                               replace(command, input_state_id=value), builder, 0)


if __name__ == '__main__':
    unittest.main()
