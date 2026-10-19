import math
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import replace
from typing import Any, TypeVar

from .transport import (
    CONTROL_ACK_TOPIC,
    CONTROL_REQUEST_TOPIC,
    CONTROL_STATUS_TOPIC,
    BODY_CMD_TOPIC,
    BODY_STATE_TOPIC,
    Transport,
    create_aorta_transport,
)
from .types import (
    ControlAck,
    ControlOperation,
    ControlRequest,
    ControlStatus,
    ExternalPhase,
    PROTOCOL_VERSION,
)

from .body_types import (
    BODY_SLOTS, BODY_PROTOCOL_VERSION, BodyState, BodyCmd, BodyJointCmd, BodyCmdAction,
    BodyInfo, GetRobotInfoResponse,
)


class ExternalControlError(RuntimeError):
    """Base exception for SDK control-session failures."""


class ExternalControlTimeoutError(ExternalControlError):
    """Raised when the robot does not reach the requested state in time."""


class ControlRejectedError(ExternalControlError):
    """Raised when the robot rejects a control request."""


class InvalidCommandError(ExternalControlError, ValueError):
    """Raised when a command is structurally invalid."""


class NotRunningError(ExternalControlError):
    """Raised when publishing without an owned EXTERNAL session."""


class ControllerClosedError(ExternalControlError):
    """Raised when an operation is attempted after close()."""


_T = TypeVar("_T")
_MAX_UINT64 = (1 << 64) - 1


class ExternalController:
    def __init__(
        self,
        *,
        group: str = "default",
        node_name: str | None = None,
        client_id: str | None = None,
        _transport: Transport | None = None,
    ) -> None:
        self.group = _nonempty_text(group, "group")
        self.node_name = _nonempty_text(
            node_name or f"locomotion-aorta-{uuid.uuid4()}", "node_name"
        )
        self.client_id = _nonempty_text(
            client_id or f"locomotion-client-{uuid.uuid4()}", "client_id"
        )
        self.transport = (
            _transport
            if _transport is not None
            else create_aorta_transport(node_name=self.node_name, group=self.group)
        )

        self._condition = threading.Condition()
        self._latest_body_state: BodyState | None = None
        self._body_info: BodyInfo | None = None
        self._latest_status: ControlStatus | None = None
        self._acks: dict[tuple[str, ControlOperation], ControlAck] = {}
        self._request_id: str | None = None
        self._lease_id = ""
        self._external_session_id = 0
        self._next_external_sequence = 1
        self._cancel_sent = False
        self._closed = False
        self._subscriptions: list[Any] = []

        try:
            self._subscriptions.append(
                self.transport.subscribe(BODY_STATE_TOPIC, self._on_body_state)
            )
            self._subscriptions.append(
                self.transport.subscribe(CONTROL_ACK_TOPIC, self._on_ack)
            )
            self._subscriptions.append(
                self.transport.subscribe(CONTROL_STATUS_TOPIC, self._on_status)
            )
        except BaseException:
            for subscription in self._subscriptions:
                try:
                    subscription.close()
                except BaseException:
                    pass
            try:
                self.transport.close()
            except BaseException:
                pass
            raise

    @property
    def status(self) -> ControlStatus | None:
        with self._condition:
            return self._latest_status

    def wait_body_state(self, timeout: float) -> BodyState:
        deadline = _deadline(timeout)
        return self._wait_for(
            lambda: self._latest_body_state,
            deadline,
            "timed out waiting for body state",
        )

    def get_robot_info(self, timeout: float = 5.0) -> GetRobotInfoResponse:
        _deadline(timeout)
        with self._condition:
            self._ensure_open()
        reply = self.transport.get_robot_info(timeout)
        with self._condition:
            self._ensure_open()
            if reply.info is not None:
                self._body_info = reply.info.body
        # Preserve partial or failed discovery and each end-effector status for the caller.
        return reply

    def enter_external(self, timeout: float) -> ControlStatus:
        """Enter EXTERNAL with no automatic expiry.

        timeout bounds entry only. Call exit_external() to release control;
        command-stream timeout and safety checks remain active.
        """
        deadline = _deadline(timeout)
        with self._condition:
            self._ensure_open()
            if self._request_id is not None:
                raise ExternalControlError("an EXTERNAL request is already active")
            request_id = str(uuid.uuid4())
            self._request_id = request_id
            self._lease_id = ""
            self._external_session_id = 0
            self._next_external_sequence = 1
            self._cancel_sent = False
            self._acks.clear()

        try:
            prepare = self._request(ControlOperation.PREPARE)
            self.transport.publish(CONTROL_REQUEST_TOPIC, prepare)
            prepare_ack = self._wait_for_ack(
                request_id, ControlOperation.PREPARE, deadline
            )
            self._require_accepted(prepare_ack)
            if not prepare_ack.lease_id:
                raise ControlRejectedError(
                    "PREPARE was accepted without a lease_id"
                )
            with self._condition:
                self._lease_id = prepare_ack.lease_id
                status_before_commit = self._latest_status

            commit = self._request(ControlOperation.COMMIT)
            self.transport.publish(CONTROL_REQUEST_TOPIC, commit)
            commit_ack = self._wait_for_ack(
                request_id, ControlOperation.COMMIT, deadline
            )
            self._require_accepted(commit_ack)
            if commit_ack.lease_id and commit_ack.lease_id != self._lease_id:
                raise ControlRejectedError(
                    "COMMIT acknowledgement changed the prepared lease_id"
                )

            status = self._wait_for(
                lambda: self._owned_running_status_after(status_before_commit),
                deadline,
                "timed out waiting for owned EXTERNAL/RUNNING status",
            )
            with self._condition:
                self._external_session_id = status.generation
                self._next_external_sequence = 1
            return status
        except BaseException:
            self._best_effort_cancel()
            raise

    def publish(self, command: BodyCmd) -> None:
        with self._condition:
            self._ensure_open()
            population = None
            if self._body_info is not None and len(self._body_info.joints) == BODY_SLOTS:
                population = tuple(j.populated for j in self._body_info.joints)
            _validate_command(command, population)
            status = self._latest_status
            if not self._status_allows_publish(status):
                raise NotRunningError(
                    "publish requires an owned EXTERNAL RUNNING or "
                    "TIMEOUT_HOLD session"
                )
            assert status is not None
            if status.phase == ExternalPhase.RUNNING:
                if not _valid_generation(status.generation):
                    raise NotRunningError(
                        "RUNNING status did not provide a valid generation"
                    )
                if status.generation != self._external_session_id:
                    self._external_session_id = status.generation
                    self._next_external_sequence = 1
            elif status.generation != self._external_session_id:
                raise NotRunningError(
                    "TIMEOUT_HOLD status does not match the learned RUNNING "
                    "generation"
                )
            if self._next_external_sequence > _MAX_UINT64:
                raise ExternalControlError("external sequence exhausted")
            external_command = replace(
                command,
                protocol_version=BODY_PROTOCOL_VERSION,
                action=BodyCmdAction.EXTERNAL,
                root_quat=None,
                session_id=self._external_session_id,
                sequence=self._next_external_sequence,
                ts_client_created_us=command.ts_client_created_us or time.monotonic_ns() // 1000,
                ts_client_send_us=time.monotonic_ns() // 1000,
            )
            self._next_external_sequence += 1
        self.transport.publish(BODY_CMD_TOPIC, external_command)

    def exit_external(self, timeout: float) -> ControlStatus:
        deadline = _deadline(timeout)
        with self._condition:
            self._ensure_open()
            if self._request_id is None:
                raise NotRunningError("no EXTERNAL request is active")
            request_id = self._request_id

        self._best_effort_cancel()
        cancel_ack = self._wait_for_ack(
            request_id, ControlOperation.CANCEL, deadline
        )
        self._require_accepted(cancel_ack)
        status = self._wait_for(
            lambda: (
                self._latest_status
                if self._latest_status is not None
                and self._latest_status.current_action != "EXTERNAL"
                and self._latest_status.phase
                in (ExternalPhase.IDLE, ExternalPhase.EMERGENCY)
                else None
            ),
            deadline,
            "timed out waiting for EXTERNAL to exit",
        )
        with self._condition:
            self._clear_session()
        return status

    def close(self) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            should_cancel = self._request_id is not None and not self._cancel_sent
            if should_cancel:
                self._cancel_sent = True
                cancel = self._request(ControlOperation.CANCEL)
            else:
                cancel = None
            self._condition.notify_all()

        if cancel is not None:
            try:
                self.transport.publish(CONTROL_REQUEST_TOPIC, cancel)
            except BaseException:
                pass
        for subscription in self._subscriptions:
            try:
                subscription.close()
            except BaseException:
                pass
        try:
            self.transport.close()
        except BaseException:
            pass

    def __enter__(self) -> "ExternalController":
        with self._condition:
            self._ensure_open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        del exc_type, exc, traceback
        self.close()

    def _on_body_state(self, lowstate: Any) -> None:
        if not isinstance(lowstate, BodyState) or lowstate.protocol_version != BODY_PROTOCOL_VERSION:
            return
        with self._condition:
            if not self._closed:
                self._latest_body_state = lowstate
                self._condition.notify_all()

    def _on_ack(self, ack: Any) -> None:
        if (
            not isinstance(ack, ControlAck)
            or ack.protocol_version != PROTOCOL_VERSION
        ):
            return
        with self._condition:
            if not self._closed and ack.request_id == self._request_id:
                key = (ack.request_id, ack.operation)
                if key not in self._acks:
                    self._acks[key] = ack
                    self._condition.notify_all()

    def _on_status(self, status: Any) -> None:
        if (
            not isinstance(status, ControlStatus)
            or status.protocol_version != PROTOCOL_VERSION
        ):
            return
        with self._condition:
            if not self._closed:
                if _status_is_newer(status, self._latest_status):
                    self._latest_status = status
                    self._condition.notify_all()

    def _wait_for_ack(
        self,
        request_id: str,
        operation: ControlOperation,
        deadline: float,
    ) -> ControlAck:
        return self._wait_for(
            lambda: self._acks.get((request_id, operation)),
            deadline,
            f"timed out waiting for {operation.value} acknowledgement",
        )

    def _wait_for(
        self,
        value: Callable[[], _T | None],
        deadline: float,
        timeout_message: str,
    ) -> _T:
        with self._condition:
            while True:
                self._ensure_open()
                result = value()
                if result is not None:
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ExternalControlTimeoutError(timeout_message)
                self._condition.wait(remaining)

    def _request(self, operation: ControlOperation) -> ControlRequest:
        if self._request_id is None:
            raise ExternalControlError("no active request identity")
        return ControlRequest(
            request_id=self._request_id,
            client_id=self.client_id,
            operation=operation,
            lease_id=self._lease_id,
            requested_lease_ms=0,
        )

    def _best_effort_cancel(self) -> None:
        with self._condition:
            if self._request_id is None or self._cancel_sent:
                return
            self._cancel_sent = True
            cancel = self._request(ControlOperation.CANCEL)
        try:
            self.transport.publish(CONTROL_REQUEST_TOPIC, cancel)
        except BaseException:
            pass

    def _owned_running_status_after(
        self, status_before_commit: ControlStatus | None
    ) -> ControlStatus | None:
        status = self._latest_status
        return (
            status
            if self._status_allows_publish(status)
            and status is not None
            and _status_is_newer(status, status_before_commit)
            and status.phase == ExternalPhase.RUNNING
            and _valid_generation(status.generation)
            else None
        )

    def _status_allows_publish(self, status: ControlStatus | None) -> bool:
        return bool(
            self._request_id is not None
            and self._lease_id
            and status is not None
            and status.current_action == "EXTERNAL"
            and status.phase in (ExternalPhase.RUNNING, ExternalPhase.TIMEOUT_HOLD)
            and status.client_id == self.client_id
            and status.lease_id == self._lease_id
        )

    def _require_accepted(self, ack: ControlAck) -> None:
        if ack.accepted:
            return
        detail = ack.reason or f"error code {ack.error_code}"
        raise ControlRejectedError(
            f"{ack.operation.value} request was rejected: {detail}"
        )

    def _clear_session(self) -> None:
        self._request_id = None
        self._lease_id = ""
        self._external_session_id = 0
        self._next_external_sequence = 1
        self._cancel_sent = False

    def _ensure_open(self) -> None:
        if self._closed:
            raise ControllerClosedError("controller is closed")


def _nonempty_text(value: str, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _deadline(timeout: float) -> float:
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise ValueError("timeout must be a finite positive number")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be a finite positive number")
    return time.monotonic() + timeout


def _valid_generation(generation: int) -> bool:
    return (
        not isinstance(generation, bool)
        and isinstance(generation, int)
        and 0 < generation <= _MAX_UINT64
    )


def _status_is_newer(
    status: ControlStatus, previous: ControlStatus | None
) -> bool:
    if previous is None:
        return True
    if status.stamp_monotonic_ns != previous.stamp_monotonic_ns and (
        status.stamp_monotonic_ns > 0 or previous.stamp_monotonic_ns > 0
    ):
        return status.stamp_monotonic_ns > previous.stamp_monotonic_ns
    return status.status_seq > previous.status_seq


def _validate_command(command: BodyCmd, populated: tuple[bool, ...] | None = None) -> None:
    if not isinstance(command, BodyCmd):
        raise InvalidCommandError("command must be a BodyCmd")
    if (isinstance(command.cmd_id, bool) or not isinstance(command.cmd_id, int)
            or not 0 < command.cmd_id < (1 << 16)):
        raise InvalidCommandError("cmd_id must be an integer in [1, 65535]")
    if not isinstance(command.joints, tuple) or len(command.joints) != BODY_SLOTS:
        raise InvalidCommandError("joints must be a 31-slot tuple")
    if command.action != BodyCmdAction.EXTERNAL:
        raise InvalidCommandError("ExternalController requires EXTERNAL action")
    for index, motor in enumerate(command.joints):
        if not isinstance(motor, BodyJointCmd):
            raise InvalidCommandError(f"joints[{index}] must be a BodyJointCmd")
        if populated is not None and not populated[index]:
            continue
        for field in ("q", "dq", "tau", "kp", "kd"):
            value = getattr(motor, field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise InvalidCommandError(f"joints[{index}].{field} must be numeric")
            if not math.isfinite(value):
                raise InvalidCommandError(f"joints[{index}].{field} must be finite")
