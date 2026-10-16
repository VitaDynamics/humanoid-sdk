"""Legacy42 IMU and inference telemetry against the pinned generated wheels."""
from dataclasses import replace
import time
import unittest

import flatbuffers
from lowlevel.HumanoidLowState import HumanoidLowState, HumanoidLowStateT
from lowlevel.HumanoidMotorState import HumanoidMotorStateT
from lowlevel.ImuSample import ImuSampleT
from lowlevel.ImuState import ImuStateT

from locomotion_aorta import LowCmd, MotorCommand, InvalidCommandError
from locomotion_aorta import transport
from locomotion_aorta.controller import _validate_command


class TrackingBindingTest(unittest.TestCase):
    def test_disabled_trunk_without_state_does_not_discard_feedback(self):
        source = HumanoidLowStateT(imuStates=[ImuSampleT(location=1, status=6)], stateId=7)
        builder = flatbuffers.Builder(128)
        builder.Finish(source.Pack(builder))
        result = transport._decode_humanoid_lowstate(HumanoidLowState.GetRootAs(builder.Output()))
        self.assertEqual(result.state_id, 7)
        self.assertEqual((result.imu_states[0].location, result.imu_states[0].status), (1, 6))
        self.assertEqual(result.imu_states[0].quaternion, ())

    def test_real_binding_decodes_hip_trunk_timestamps_and_44_motors(self):
        hip = ImuSampleT(location=0, status=1, tsSampledUs=100, tsAcquiredHwUs=101,
                         tsIngestedUs=102, tsAssembledUs=103,
                         state=ImuStateT([1., 0., 0., 0.], [1., 2., 3.], [0., 0., 9.8], [0., 0., 0.]))
        trunk = ImuSampleT(location=1, status=6,
                           state=ImuStateT([0.]*4, [0.]*3, [0.]*3, [0.]*3))
        source = HumanoidLowStateT(imuStates=[trunk, hip], stateId=65535,
                                   tsStateReal=1234567890123456, tsStatePub=1234567890123500,
                                   tsConsumedUs=1234567890123490, attitudeValid=True,
                                   motorStates=[HumanoidMotorStateT(q=i/4, dq=-i/2) for i in range(44)])
        builder = flatbuffers.Builder(1024)
        builder.Finish(source.Pack(builder))
        before = time.monotonic_ns()
        result = transport._decode_humanoid_lowstate(HumanoidLowState.GetRootAs(builder.Output()))
        self.assertEqual(result.stamp_ns, source.tsStateReal*1000)
        self.assertEqual((result.ts_state_pub_us, result.ts_consumed_us),
                         (source.tsStatePub, source.tsConsumedUs))
        self.assertGreaterEqual(result.received_monotonic_ns, before)
        self.assertLessEqual(result.received_monotonic_ns, time.monotonic_ns())
        self.assertEqual((result.imu_states[0].location, result.imu_states[0].status), (1, 6))
        self.assertEqual(result.imu_states[1].quaternion, (1., 0., 0., 0.))
        self.assertEqual(result.imu_states[1].gyroscope, (1., 2., 3.))
        self.assertEqual((result.imu_states[1].ts_sampled_us, result.imu_states[1].ts_ingested_us), (100, 102))
        self.assertEqual(len(result.motor_states), 44)
        self.assertEqual((result.motor_states[43].q, result.motor_states[43].dq), (10.75, -21.5))

    def test_real_binding_encodes_input_and_inference_metadata(self):
        command = LowCmd(65535, (MotorCommand(10, .25),)*42,
                         input_state_id=65000, input_state_ts_real=1234567890123456,
                         ts_infer_input_state=1234567890124000,
                         ts_before_infer=1234567890124500, ts_after_infer=1234567890139500,
                         max_abs_obs=12.5)
        _validate_command(command)
        bindings = transport._load_aorta_bindings()
        builder = flatbuffers.Builder(1024)
        builder.Finish(transport._fill_low_cmd(bindings, command, builder, 0))
        view = bindings.low_cmd_module.LowCmd.GetRootAs(builder.Output())
        for name, expected in (('InputStateId', 65000), ('InputStateTsReal', command.input_state_ts_real),
                               ('TsInferInputState', command.ts_infer_input_state),
                               ('TsBeforeInfer', command.ts_before_infer),
                               ('TsAfterInfer', command.ts_after_infer), ('MaxAbsObs', 12.5)):
            self.assertEqual(getattr(view, name)(), expected)
        self.assertEqual(view.MotorCmdLength(), 42)
        self.assertEqual(view.MotorCmd(41).Q(), .25)

    def test_invalid_metadata_rejected_before_transport(self):
        command = LowCmd(1, (MotorCommand(10),)*42)
        for key, value in (('input_state_id', -1), ('input_state_id', 65536),
                           ('input_state_id', True), ('ts_before_infer', 2**64),
                           ('input_state_ts_real', 1.5), ('max_abs_obs', -1.),
                           ('max_abs_obs', float('nan')), ('max_abs_obs', 1e100)):
            with self.subTest(key=key, value=value), self.assertRaises(InvalidCommandError):
                _validate_command(replace(command, **{key:value}))


if __name__ == '__main__':
    unittest.main()
