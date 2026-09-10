from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from .types import (
    ControlAck,
    ControlOperation,
    ControlRequest,
    ControlStatus,
    ExternalPhase,
    HumanoidLowState,
    LowCmd,
    MotorState,
)


LOWSTATE_TOPIC = "/sdk/foot_humanoid/lowstate"
CONTROL_REQUEST_TOPIC = "/sdk/foot_humanoid/control/request"
CONTROL_ACK_TOPIC = "/sdk/foot_humanoid/control/ack"
CONTROL_STATUS_TOPIC = "/sdk/foot_humanoid/control/status"
EXTERNAL_COMMAND_TOPIC = "/locomotion/external_command"


class AortaBindingsUnavailableError(RuntimeError):
    """Raised when the compatible Aorta Python bindings are unavailable."""


class Subscription(Protocol):
    def close(self) -> None: ...


class Transport(Protocol):
    def subscribe(
        self, topic: str, callback: Callable[[Any], None]
    ) -> Subscription: ...

    def publish(self, topic: str, message: Any) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class _AortaBindings:
    aorta: Any
    control_request_module: Any
    low_cmd_module: Any
    motor_cmd_module: Any
    control_request_schema_meta: Any
    low_cmd_schema_meta: Any
    humanoid_low_state_decoder: Callable[[bytes], Any]
    control_ack_decoder: Callable[[bytes], Any]
    control_status_decoder: Callable[[bytes], Any]


class _AortaTransport:
    def __init__(self, *, node_name: str, group: str) -> None:
        self._bindings = _load_aorta_bindings()
        self.node = self._bindings.aorta.Node(node_name, group)
        self.publishers: dict[str, Any] = {}
        self.subscriptions: list[Any] = []
        self._closed = False
        try:
            realtime = self._bindings.aorta.QoS.realtime_control()
            self.publishers[CONTROL_REQUEST_TOPIC] = (
                self.node.create_publisher_typed(
                    self._bindings.control_request_schema_meta,
                    CONTROL_REQUEST_TOPIC,
                    qos=realtime,
                )
            )
            self.publishers[EXTERNAL_COMMAND_TOPIC] = (
                self.node.create_publisher_typed(
                    self._bindings.low_cmd_schema_meta,
                    EXTERNAL_COMMAND_TOPIC,
                    qos=realtime,
                )
            )
        except BaseException:
            self.node.close()
            raise

    def subscribe(
        self, topic: str, callback: Callable[[Any], None]
    ) -> Subscription:
        if self._closed:
            raise RuntimeError("Aorta transport is closed")
        decoder, convert, qos = self._subscriber_spec(topic)
        subscription = self.node.create_subscriber_typed(
            decoder,
            topic,
            lambda view, _context: callback(convert(view)),
            qos=qos,
        )
        self.subscriptions.append(subscription)
        return subscription

    def publish(self, topic: str, message: Any) -> None:
        if self._closed:
            raise RuntimeError("Aorta transport is closed")
        publisher = self.publishers.get(topic)
        if publisher is None:
            raise ValueError(f"unsupported publish topic: {topic}")
        if topic == CONTROL_REQUEST_TOPIC:
            if not isinstance(message, ControlRequest):
                raise TypeError("control request topic requires ControlRequest")
            publisher.publish_typed(
                lambda builder, header: _fill_control_request(
                    self._bindings, message, builder, header
                )
            )
            return
        if not isinstance(message, LowCmd):
            raise TypeError("external command topic requires LowCmd")
        publisher.publish_typed(
            lambda builder, header: _fill_low_cmd(
                self._bindings, message, builder, header
            )
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.node.close()

    def _subscriber_spec(
        self, topic: str
    ) -> tuple[Callable[[bytes], Any], Callable[[Any], Any], Any]:
        if topic == LOWSTATE_TOPIC:
            return (
                self._bindings.humanoid_low_state_decoder,
                _decode_humanoid_lowstate,
                self._bindings.aorta.QoS.sensor_data(),
            )
        if topic == CONTROL_ACK_TOPIC:
            return (
                self._bindings.control_ack_decoder,
                _decode_control_ack,
                self._bindings.aorta.QoS.realtime_control(),
            )
        if topic == CONTROL_STATUS_TOPIC:
            return (
                self._bindings.control_status_decoder,
                _decode_control_status,
                self._bindings.aorta.QoS.state_update(),
            )
        raise ValueError(f"unsupported subscription topic: {topic}")


def create_aorta_transport(*, node_name: str, group: str) -> Transport:
    try:
        return _AortaTransport(node_name=node_name, group=group)
    except (ImportError, OSError) as error:
        raise AortaBindingsUnavailableError(
            "the compatible Aorta Python control bindings are unavailable; "
            "complete the M0 binding gate and install the pinned Aorta wheels"
        ) from error


def _load_aorta_bindings() -> _AortaBindings:
    aorta = importlib.import_module("aorta")
    control_request = importlib.import_module("locomotion_sdk.ControlRequest")
    control_ack = importlib.import_module("locomotion_sdk.ControlAck")
    control_status = importlib.import_module("locomotion_sdk.ControlStatus")
    humanoid_lowstate = importlib.import_module("lowlevel.HumanoidLowState")
    low_cmd = importlib.import_module("lowlevel.LowCmd")
    motor_cmd = importlib.import_module("lowlevel.MotorCmd")
    return _AortaBindings(
        aorta=aorta,
        control_request_module=control_request,
        low_cmd_module=low_cmd,
        motor_cmd_module=motor_cmd,
        control_request_schema_meta=importlib.import_module(
            "control_request_schema_meta"
        ),
        low_cmd_schema_meta=importlib.import_module("low_cmd_schema_meta"),
        humanoid_low_state_decoder=humanoid_lowstate.HumanoidLowState.GetRootAs,
        control_ack_decoder=control_ack.ControlAck.GetRootAs,
        control_status_decoder=control_status.ControlStatus.GetRootAs,
    )


def _fill_control_request(
    bindings: _AortaBindings,
    request: ControlRequest,
    builder: Any,
    header: int,
) -> int:
    module = bindings.control_request_module
    request_id = builder.CreateString(request.request_id)
    client_id = builder.CreateString(request.client_id)
    action_name = builder.CreateString(request.action_name)
    lease_id = builder.CreateString(request.lease_id)
    module.ControlRequestStart(builder)
    module.ControlRequestAddAortaHeader(builder, header)
    module.ControlRequestAddProtocolVersion(builder, request.protocol_version)
    module.ControlRequestAddOperation(builder, _encode_operation(request.operation))
    module.ControlRequestAddRequestId(builder, request_id)
    module.ControlRequestAddClientId(builder, client_id)
    module.ControlRequestAddActionName(builder, action_name)
    module.ControlRequestAddLeaseId(builder, lease_id)
    module.ControlRequestAddRequestedLeaseMs(builder, request.requested_lease_ms)
    return module.ControlRequestEnd(builder)


def _fill_low_cmd(
    bindings: _AortaBindings,
    command: LowCmd,
    builder: Any,
    header: int,
) -> int:
    module = bindings.low_cmd_module
    module.LowCmdStartMotorCmdVector(builder, len(command.motor_cmds))
    for motor in reversed(command.motor_cmds):
        bindings.motor_cmd_module.CreateMotorCmd(
            builder,
            motor.mode,
            motor.q,
            motor.dq,
            motor.tau,
            motor.kp,
            motor.kd,
        )
    motor_cmds = builder.EndVector()
    module.LowCmdStart(builder)
    module.LowCmdAddAortaHeader(builder, header)
    module.LowCmdAddMotorCmd(builder, motor_cmds)
    module.LowCmdAddCmdId(builder, command.cmd_id)
    module.LowCmdAddExternalVersion(builder, command.external_version)
    module.LowCmdAddExternalSessionId(builder, command.external_session_id)
    module.LowCmdAddExternalSequence(builder, command.external_sequence)
    return module.LowCmdEnd(builder)


def _decode_control_ack(view: Any) -> ControlAck:
    return ControlAck(
        request_id=_decode_text(view.RequestId()),
        operation=_decode_operation(view.Operation()),
        accepted=bool(view.Accepted()),
        error_code=int(view.ErrorCode()),
        reason=_decode_text(view.Reason()),
        lease_id=_decode_text(view.LeaseId()),
        lease_deadline_monotonic_ns=int(view.LeaseDeadlineMonotonicNs()),
        protocol_version=int(view.ProtocolVersion()),
    )


def _decode_control_status(view: Any) -> ControlStatus:
    return ControlStatus(
        status_seq=int(view.StatusSeq()),
        current_action=_decode_text(view.CurrentAction()),
        phase=_decode_phase(view.ActionPhase()),
        client_id=_decode_text(view.ClientId()),
        lease_id=_decode_text(view.LeaseId()),
        lease_deadline_monotonic_ns=int(view.LeaseDeadlineMonotonicNs()),
        generation=int(view.Generation()),
        last_accepted_cmd_id=int(view.LastAcceptedCmdId()),
        last_valid_age_ms=int(view.LastValidAgeMs()),
        estimated_input_period_us=int(view.EstimatedInputPeriodUs()),
        timeout_hold=bool(view.TimeoutHold()),
        last_reject_code=int(view.LastRejectCode()),
        reject_count=int(view.RejectCount()),
        ingress_drop_count=int(view.IngressDropCount()),
        stamp_monotonic_ns=int(view.StampMonotonicNs()),
        protocol_version=int(view.ProtocolVersion()),
    )


def _decode_humanoid_lowstate(view: Any) -> HumanoidLowState:
    motor_states = []
    for index in range(view.MotorStatesLength()):
        motor = view.MotorStates(index)
        motor_states.append(
            MotorState()
            if motor is None
            else MotorState(
                q=float(motor.Q()),
                dq=float(motor.Dq()),
                tau_est=float(motor.TauEst()),
                temperature=float(motor.Temperature()),
            )
        )
    return HumanoidLowState(
        state_id=int(view.StateId()),
        stamp_ns=int(view.TsStateReal()) * 1000,
        motor_states=tuple(motor_states),
        attitude_valid=bool(view.AttitudeValid()),
    )


def _encode_operation(operation: ControlOperation) -> int:
    return {
        ControlOperation.PREPARE: 1,
        ControlOperation.COMMIT: 2,
        ControlOperation.CANCEL: 3,
    }[operation]


def _decode_operation(value: int) -> ControlOperation:
    try:
        return {
            1: ControlOperation.PREPARE,
            2: ControlOperation.COMMIT,
            3: ControlOperation.CANCEL,
        }[int(value)]
    except KeyError as error:
        raise ValueError(f"unknown ControlOperation value: {value}") from error


def _decode_phase(value: int) -> ExternalPhase:
    phases = (
        ExternalPhase.IDLE,
        ExternalPhase.PREPARING,
        ExternalPhase.ARMED,
        ExternalPhase.TRANSITIONING,
        ExternalPhase.RUNNING,
        ExternalPhase.TIMEOUT_HOLD,
        ExternalPhase.CANCELLED,
        ExternalPhase.EXPIRED,
        ExternalPhase.EMERGENCY,
    )
    try:
        index = int(value)
        if index < 0:
            raise IndexError
        return phases[index]
    except (IndexError, TypeError, ValueError) as error:
        raise ValueError(f"unknown ControlActionPhase value: {value}") from error


def _decode_text(value: bytes | str | None) -> str:
    if value is None:
        return ""
    return value.decode("utf-8") if isinstance(value, bytes) else value
