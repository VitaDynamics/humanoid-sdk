import importlib
import unittest

import locomotion_aorta.transport as transport_module
from locomotion_aorta.body_types import DescriptorRef
from locomotion_aorta import (
    CONTROL_ACK_TOPIC,
    CONTROL_REQUEST_TOPIC,
    CONTROL_STATUS_TOPIC,
    BODY_CMD_TOPIC,
    BODY_STATE_TOPIC,
    ControlOperation,
    ControlRequest,
    BodyCmd,
    BodyJointCmd,
)


class AortaBindingContractTest(unittest.TestCase):
    def test_real_control_codec_preserves_both_bound_actions_for_every_operation(self):
        import flatbuffers

        bindings = transport_module._load_aorta_bindings()
        for action in ('EXTERNAL', 'HUMANOID_TELEOP'):
            for operation in ControlOperation:
                with self.subTest(action=action, operation=operation):
                    request = ControlRequest(request_id='bound-request', client_id=9,
                                             operation=operation, action_name=action, control_id=13,
                                             descriptor_ref=DescriptorRef(11))
                    builder = flatbuffers.Builder(256)
                    builder.Finish(transport_module._fill_control_request(bindings, request, builder, 0))
                    view = bindings.control_request_module.ControlRequest.GetRootAs(builder.Output())
                    self.assertEqual(view.ActionName(), action.encode())
                    self.assertEqual(view.RequestId(), b'bound-request')
                    self.assertEqual(view.ClientId(), 9)
                    self.assertEqual(view.ControlId(), 13)
                    self.assertEqual(view.DescriptorRef().RobotRuntimeId(), 11)
                    self.assertEqual(view.RequestedLeaseMs(), 0)
                    self.assertEqual(view.ProtocolVersion(), 3)
                    self.assertEqual(view.Operation(), list(ControlOperation).index(operation) + 1)

    def test_real_aorta_and_generated_control_bindings_are_available(self) -> None:
        required_modules = (
            "aorta",
            "lowlevel.BodyState",
            "lowlevel.BodyCmd",
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
            "lowlevel.BodyState": (
                "ProtocolVersion", "RosterVersion", "StateId", "TsStateRealUs",
                "TsStatePubUs", "TsConsumedUs", "AttitudeValid", "ImuStates",
                "Joints", "AppliedSessionId", "AppliedSequence", "TsFirstOutputUs",
            ),
            "lowlevel.BodyCmd": (
                "Joints", "ProtocolVersion", "SessionId", "Sequence",
                "InputStateId", "InputStateTsRealUs", "Action", "RootQuat",
            ),
            "locomotion_sdk.ControlRequest": (
                "ProtocolVersion",
                "Operation",
                "RequestId",
                "ClientId",
                "ActionName",
                "ControlId",
                "DescriptorRef",
                "RequestedLeaseMs",
            ),
            "locomotion_sdk.ControlAck": (
                "ProtocolVersion",
                "Operation",
                "RequestId",
                "Accepted",
                "ErrorCode",
                "Reason",
                "ControlId",
                "ClientId",
                "DescriptorRef",
                "LeaseDeadlineMonotonicNs",
            ),
            "locomotion_sdk.ControlStatus": (
                "ProtocolVersion",
                "StatusSeq",
                "CurrentAction",
                "ActionPhase",
                "ClientId",
                "ControlId",
                "DescriptorRef",
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
            client_id=9,
            operation=ControlOperation.COMMIT,
            control_id=13,
            descriptor_ref=DescriptorRef(11),
        )
        builder = flatbuffers.Builder(256)
        root = transport_module._fill_control_request(
            bindings, request, builder, 0
        )
        builder.Finish(root)
        request_view = bindings.control_request_module.ControlRequest.GetRootAs(
            builder.Output()
        )
        self.assertEqual(request_view.ProtocolVersion(), 3)
        self.assertEqual(request_view.Operation(), 2)
        self.assertEqual(request_view.RequestId(), b"request-1")
        self.assertEqual(request_view.ClientId(), 9)
        self.assertEqual(request_view.ControlId(), 13)
        self.assertEqual(request_view.DescriptorRef().RobotRuntimeId(), 11)
        self.assertEqual(request_view.RequestedLeaseMs(), 0)

        command = BodyCmd(
            joints=tuple(BodyJointCmd(q=0.25, kp=30.0, kd=1.5) for _ in range(31)),
            protocol_version=2,
            session_id=42,
            sequence=3,
        )
        builder = flatbuffers.Builder(256)
        root = transport_module._fill_body_cmd(bindings, command, builder, 0)
        builder.Finish(root)
        command_view = bindings.body_cmd_module.BodyCmd.GetRootAs(builder.Output())
        self.assertFalse(hasattr(command_view, 'CmdId'))
        self.assertEqual(command_view.JointsLength(), 31)
        self.assertAlmostEqual(command_view.Joints(0).Q(), 0.25)
        self.assertEqual(command_view.ProtocolVersion(), 2)
        self.assertEqual(command_view.SessionId(), 42)
        self.assertEqual(command_view.Sequence(), 3)

    def test_prepare_encodes_zero_duration_with_real_bindings(self) -> None:
        import flatbuffers

        bindings = transport_module._load_aorta_bindings()
        request = ControlRequest(
            request_id="request-unlimited",
            client_id=7,
            operation=ControlOperation.PREPARE,
            descriptor_ref=DescriptorRef(11),
        )
        builder = flatbuffers.Builder(256)
        root = transport_module._fill_control_request(bindings, request, builder, 0)
        builder.Finish(root)
        view = bindings.control_request_module.ControlRequest.GetRootAs(builder.Output())
        self.assertEqual(view.Operation(), 1)
        self.assertEqual(view.RequestId(), b"request-unlimited")
        self.assertEqual(view.ClientId(), 7)
        self.assertEqual(view.ControlId(), 0)
        self.assertEqual(view.DescriptorRef().RobotRuntimeId(), 11)
        self.assertEqual(view.RequestedLeaseMs(), 0)

    def test_body_timestamp_preserves_the_robot_microsecond_clock(self) -> None:
        import lowlevel.BodyState as module
        from test_body_codec import make_state
        payload, _ = make_state()
        state = transport_module.decode_body_state(module.BodyState.GetRootAs(payload))
        self.assertEqual(state.ts_state_real_us, 1_234_567)
        self.assertEqual(len(state.joints), 31)

    def test_public_topic_constants_match_the_protocol(self) -> None:
        self.assertEqual(BODY_STATE_TOPIC, "/sdk/foot_humanoid/body_state")
        self.assertEqual(
            CONTROL_REQUEST_TOPIC, "/sdk/foot_humanoid/control/request"
        )
        self.assertEqual(CONTROL_ACK_TOPIC, "/sdk/foot_humanoid/control/ack")
        self.assertEqual(CONTROL_STATUS_TOPIC, "/sdk/foot_humanoid/control/status")
        self.assertEqual(BODY_CMD_TOPIC, "/sdk/foot_humanoid/body_cmd")


if __name__ == "__main__":
    unittest.main()

class R3ControlIdentityBindingTest(unittest.TestCase):
    def test_actual_control_fill_rejects_invalid_numeric_and_runtime_before_encoding(self):
        import flatbuffers
        from dataclasses import replace
        bindings = transport_module._load_aorta_bindings()
        valid = ControlRequest(request_id='one', client_id=9,
                               operation=ControlOperation.COMMIT, control_id=13,
                               descriptor_ref=DescriptorRef(11))
        for changes in ({'client_id': True}, {'client_id': 0}, {'client_id': -1},
                        {'client_id': 1 << 64}, {'control_id': True},
                        {'control_id': 0}, {'control_id': 1 << 64},
                        {'descriptor_ref': None}, {'descriptor_ref': DescriptorRef(0)}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                transport_module._fill_control_request(
                    bindings, replace(valid, **changes), flatbuffers.Builder(256), 0)
