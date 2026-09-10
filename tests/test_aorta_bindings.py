import importlib
import unittest

import locomotion_aorta.transport as transport_module
from locomotion_aorta import (
    CONTROL_ACK_TOPIC,
    CONTROL_REQUEST_TOPIC,
    CONTROL_STATUS_TOPIC,
    EXTERNAL_COMMAND_TOPIC,
    LOWSTATE_TOPIC,
    ControlOperation,
    ControlRequest,
    LowCmd,
    MotorCommand,
)


class AortaBindingContractTest(unittest.TestCase):
    def test_real_aorta_and_generated_control_bindings_are_available(self) -> None:
        required_modules = (
            "aorta",
            "lowlevel.HumanoidLowState",
            "lowlevel.LowCmd",
            "locomotion_sdk.ControlRequest",
            "locomotion_sdk.ControlAck",
            "locomotion_sdk.ControlStatus",
        )
        missing = []
        modules = {}
        for module_name in required_modules:
            try:
                modules[module_name] = importlib.import_module(module_name)
            except ImportError as error:
                missing.append(f"{module_name} ({error})")
        self.assertEqual(
            missing,
            [],
            "M0 Aorta Python bindings are unavailable: " + ", ".join(missing),
        )

        required_accessors = {
            "lowlevel.HumanoidLowState": (
                "ImuStates",
                "MotorStates",
                "StateId",
                "TsStateReal",
                "TsStatePub",
                "InputCmdId",
                "AttitudeValid",
                "TsConsumedUs",
                "CmdId",
                "InputStateId",
                "InputStateTsReal",
                "TsInferInputState",
                "TsInputCmdIntoMotor",
                "TsBeforeInfer",
                "TsAfterInfer",
                "MaxAbsObs",
                "RelativeMotorCmd",
            ),
            "lowlevel.LowCmd": (
                "MotorCmd",
                "CmdId",
                "ExternalVersion",
                "ExternalSessionId",
                "ExternalSequence",
            ),
            "locomotion_sdk.ControlRequest": (
                "ProtocolVersion",
                "Operation",
                "RequestId",
                "ClientId",
                "ActionName",
                "LeaseId",
                "RequestedLeaseMs",
            ),
            "locomotion_sdk.ControlAck": (
                "ProtocolVersion",
                "Operation",
                "RequestId",
                "Accepted",
                "ErrorCode",
                "Reason",
                "LeaseId",
                "LeaseDeadlineMonotonicNs",
            ),
            "locomotion_sdk.ControlStatus": (
                "ProtocolVersion",
                "StatusSeq",
                "CurrentAction",
                "ActionPhase",
                "ClientId",
                "LeaseId",
                "LeaseDeadlineMonotonicNs",
                "Generation",
                "LastAcceptedCmdId",
                "LastValidAgeMs",
                "EstimatedInputPeriodUs",
                "TimeoutHold",
                "LastRejectCode",
                "RejectCount",
                "IngressDropCount",
                "StampMonotonicNs",
            ),
        }
        missing_accessors = []
        for module_name, accessors in required_accessors.items():
            class_name = module_name.rsplit(".", 1)[-1]
            binding = getattr(modules[module_name], class_name, None)
            if binding is None:
                missing_accessors.append(f"{module_name}.{class_name}")
                continue
            missing_accessors.extend(
                f"{module_name}.{accessor}"
                for accessor in accessors
                if not hasattr(binding, accessor)
            )
        self.assertEqual(
            missing_accessors,
            [],
            "M0 Aorta Python binding fields are incomplete: "
            + ", ".join(missing_accessors),
        )

    def test_transport_encoders_use_the_generated_message_contract(self) -> None:
        import flatbuffers

        bindings = transport_module._load_aorta_bindings()
        self.assertTrue(hasattr(bindings.aorta, "Node"))
        self.assertTrue(hasattr(bindings.aorta, "QoS"))

        request = ControlRequest(
            request_id="request-1",
            client_id="client-1",
            operation=ControlOperation.COMMIT,
            lease_id="lease-1",
        )
        builder = flatbuffers.Builder(256)
        root = transport_module._fill_control_request(
            bindings, request, builder, 0
        )
        builder.Finish(root)
        request_view = bindings.control_request_module.ControlRequest.GetRootAs(
            builder.Output()
        )
        self.assertEqual(request_view.Operation(), 2)
        self.assertEqual(request_view.RequestId(), b"request-1")
        self.assertEqual(request_view.LeaseId(), b"lease-1")
        self.assertEqual(request_view.RequestedLeaseMs(), 0)

        command = LowCmd(
            cmd_id=17,
            motor_cmds=(MotorCommand(mode=10, q=0.25, kp=30.0, kd=1.5),),
            external_version=1,
            external_session_id=42,
            external_sequence=3,
        )
        builder = flatbuffers.Builder(256)
        root = transport_module._fill_low_cmd(bindings, command, builder, 0)
        builder.Finish(root)
        command_view = bindings.low_cmd_module.LowCmd.GetRootAs(builder.Output())
        self.assertEqual(command_view.CmdId(), 17)
        self.assertEqual(command_view.MotorCmdLength(), 1)
        self.assertAlmostEqual(command_view.MotorCmd(0).Q(), 0.25)
        self.assertEqual(command_view.ExternalVersion(), 1)
        self.assertEqual(command_view.ExternalSessionId(), 42)
        self.assertEqual(command_view.ExternalSequence(), 3)

    def test_lowstate_timestamp_converts_microseconds_to_nanoseconds(self) -> None:
        import flatbuffers
        import lowlevel.HumanoidLowState as lowstate_module

        builder = flatbuffers.Builder(128)
        lowstate_module.HumanoidLowStateStart(builder)
        lowstate_module.HumanoidLowStateAddTsStateReal(builder, 1_234_567)
        builder.Finish(lowstate_module.HumanoidLowStateEnd(builder))
        view = lowstate_module.HumanoidLowState.GetRootAs(builder.Output())
        state = transport_module._decode_humanoid_lowstate(view)
        self.assertEqual(state.stamp_ns, 1_234_567_000)

    def test_public_topic_constants_match_the_protocol(self) -> None:
        self.assertEqual(LOWSTATE_TOPIC, "/sdk/foot_humanoid/lowstate")
        self.assertEqual(
            CONTROL_REQUEST_TOPIC, "/sdk/foot_humanoid/control/request"
        )
        self.assertEqual(CONTROL_ACK_TOPIC, "/sdk/foot_humanoid/control/ack")
        self.assertEqual(CONTROL_STATUS_TOPIC, "/sdk/foot_humanoid/control/status")
        self.assertEqual(EXTERNAL_COMMAND_TOPIC, "/locomotion/external_command")


if __name__ == "__main__":
    unittest.main()
