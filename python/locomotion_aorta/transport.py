from __future__ import annotations

import importlib
import os
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from ._body_codec import create_body_cmd, decode_body_state, decode_robot_info_response
from .body_types import BodyCmd, GetRobotInfoResponse
from .types import (
    ControlAck,
    ControlOperation,
    ControlRequest,
    ControlStatus,
    ExternalPhase,
)


BODY_STATE_TOPIC = "/sdk/foot_humanoid/body_state"
BODY_CMD_TOPIC = "/sdk/foot_humanoid/body_cmd"
GET_ROBOT_INFO_SERVICE = "/sdk/foot_humanoid/get_robot_info"
CONTROL_REQUEST_TOPIC = "/sdk/foot_humanoid/control/request"
CONTROL_ACK_TOPIC = "/sdk/foot_humanoid/control/ack"
CONTROL_STATUS_TOPIC = "/sdk/foot_humanoid/control/status"


class AortaBindingsUnavailableError(RuntimeError):
    """Raised when the compatible Aorta Python bindings are unavailable."""


class Subscription(Protocol):
    def close(self) -> None: ...


class Transport(Protocol):
    def subscribe(
        self, topic: str, callback: Callable[[Any], None]
    ) -> Subscription: ...

    def publish(self, topic: str, message: Any) -> None: ...

    def get_robot_info(self, timeout: float) -> GetRobotInfoResponse: ...

    def close(self) -> None: ...


@dataclass(frozen=True, slots=True)
class _AortaBindings:
    aorta: Any
    control_request_module: Any
    body_cmd_module: Any
    control_request_schema_meta: Any
    body_cmd_schema_meta: Any
    body_state_decoder: Callable[[bytes], Any]
    get_robot_info_request: Any
    get_robot_info_response: Any
    get_robot_info_schema_meta: Any
    control_ack_decoder: Callable[[bytes], Any]
    control_status_decoder: Callable[[bytes], Any]


class _AortaTransport:
    def __init__(self, *, node_name: str, group: str) -> None:
        self._bindings = _load_aorta_bindings()
        self.node = self._bindings.aorta.Node(node_name, group)
        self.publishers: dict[str, Any] = {}
        self.subscriptions: list[Any] = []
        self._info_clients: dict[int, Any] = {}
        self._info_lock = threading.Lock()
        self._closed = False
        self._capture_publish_timing = os.environ.get("LOCOMOTION_AORTA_PUBLISH_TIMING") == "1"
        self._publish_trace = threading.local()
        self._body_publisher = None
        try:
            realtime = self._bindings.aorta.QoS.realtime_control()
            self.publishers[CONTROL_REQUEST_TOPIC] = (
                self.node.create_publisher_typed(
                    self._bindings.control_request_schema_meta,
                    CONTROL_REQUEST_TOPIC,
                    qos=realtime,
                )
            )
            self.publishers[BODY_CMD_TOPIC] = (
                self.node.create_publisher_typed(
                    self._bindings.body_cmd_schema_meta,
                    BODY_CMD_TOPIC,
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
        self._publish_trace.last = None
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
        if not isinstance(message, BodyCmd):
            raise TypeError("body command topic requires BodyCmd")
        from ._native_body_codec import NativeBodyPublisher
        if self._body_publisher is None:
            self._body_publisher = NativeBodyPublisher(publisher, self._bindings.body_cmd_schema_meta)
        trace = {} if self._capture_publish_timing else None
        self._publish_trace.last = trace
        self._body_publisher.publish(message, trace)

    def _last_publish_timing(self):
        trace = getattr(self._publish_trace, "last", None)
        return dict(trace) if trace is not None else None

    def get_robot_info(self, timeout: float) -> GetRobotInfoResponse:
        with self._info_lock:
            return self._get_robot_info_locked(timeout)

    def _get_robot_info_locked(self, timeout: float) -> GetRobotInfoResponse:
        import math
        if self._closed:
            raise RuntimeError("Aorta transport is closed")
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        timeout_ms = math.ceil(timeout * 1000)
        if timeout_ms not in self._info_clients:
            self._info_clients[timeout_ms] = self.node.create_client_typed(
                GET_ROBOT_INFO_SERVICE,
                self._bindings.get_robot_info_response.GetRobotInfoResponse.GetRootAs,
                request_schema_meta=self._bindings.get_robot_info_schema_meta,
                options=self._bindings.aorta.ClientOptions(timeout_ms=timeout_ms))
        client = self._info_clients[timeout_ms]
        if not client.raw().wait_for_matching(timeout_ms):
            raise TimeoutError("get_robot_info service provider not found")
        reply = client.call(lambda builder, header: _fill_robot_info_request(
            self._bindings.get_robot_info_request, builder, header)).decode()
        return decode_robot_info_response(reply)

    def close(self) -> None:
        with self._info_lock:
            if self._closed:
                return
            self._closed = True
            self.node.close()

    def _subscriber_spec(
        self, topic: str
    ) -> tuple[Callable[[bytes], Any], Callable[[Any], Any], Any]:
        if topic == BODY_STATE_TOPIC:
            return (
                self._bindings.body_state_decoder,
                decode_body_state,
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
    body_state = importlib.import_module("lowlevel.BodyState")
    body_cmd = importlib.import_module("lowlevel.BodyCmd")
    return _AortaBindings(
        aorta=aorta,
        control_request_module=control_request,
        body_cmd_module=body_cmd,
        control_request_schema_meta=importlib.import_module(
            "control_request_schema_meta"
        ),
        body_cmd_schema_meta=importlib.import_module("body_cmd_schema_meta"),
        body_state_decoder=body_state.BodyState.GetRootAs,
        get_robot_info_request=importlib.import_module("aorta.services.humanoid_locomotion.GetRobotInfoRequest"),
        get_robot_info_response=importlib.import_module("aorta.services.humanoid_locomotion.GetRobotInfoResponse"),
        get_robot_info_schema_meta=importlib.import_module("get_robot_info_schema_meta"),
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


def _fill_body_cmd(
    bindings: _AortaBindings, command: BodyCmd, builder: Any, header: int,
) -> int:
    module = bindings.body_cmd_module
    joints = create_body_cmd(builder, command.joints)
    module.BodyCmdStart(builder)
    module.BodyCmdAddAortaHeader(builder, header)
    module.BodyCmdAddJoints(builder, joints)
    for name in ("protocol_version", "session_id", "sequence", "input_state_id",
                 "input_state_ts_real_us", "ts_client_created_us", "ts_client_send_us"):
        getter = ''.join(part.capitalize() for part in name.split('_'))
        getattr(module, 'BodyCmdAdd' + getter)(builder, getattr(command, name))
    # Action is present even when equal to the schema default. Keep other defaults unchanged.
    previous = builder.forceDefaults
    builder.ForceDefaults(True)
    try:
        module.BodyCmdAddAction(builder, command.action)
    finally:
        builder.ForceDefaults(previous)
    if command.root_quat is not None:
        from lowlevel.RootQuatWxyz import CreateRootQuatWxyz
        module.BodyCmdAddRootQuat(builder, CreateRootQuatWxyz(builder, *command.root_quat))
    return module.BodyCmdEnd(builder)


def _fill_robot_info_request(module: Any, builder: Any, header: int) -> int:
    module.GetRobotInfoRequestStart(builder)
    module.GetRobotInfoRequestAddAortaHeader(builder, header)
    return module.GetRobotInfoRequestEnd(builder)


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
