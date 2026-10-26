import math
import operator
import os
import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass, replace
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
    BODY_SLOTS, BODY_CMD_PROTOCOL_VERSION, BODY_STATE_PROTOCOL_VERSION, BodyState, BodyCmd, BodyJointCmd, BodyCmdAction,
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
    """Raised when an operation lacks a confirmed, owned control session."""


class ControllerClosedError(ExternalControlError):
    """Raised when an operation is attempted after close()."""


_T = TypeVar("_T")
_MAX_UINT64 = (1 << 64) - 1


@dataclass(frozen=True, slots=True)
class _CommandContext:
    """One call's confirmed connection context, never an ownership credential."""

    client_id: str
    lease_id: str
    session_id: int
    sequence: int


@dataclass(frozen=True, slots=True)
class _ControlContext:
    """Local RPC authorization snapshot; generation/request are never wire credentials."""

    client_id: str
    lease_id: str
    generation: int
    request_id: str


class ExternalController:
    def __init__(
        self,
        *,
        group: str = "default",
        node_name: str | None = None,
        client_id: str | None = None,
        _transport: Transport | None = None,
        _subscribe_body_state: bool = True,
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
        self._capture_publish_timing = os.environ.get("LOCOMOTION_AORTA_PUBLISH_TIMING") == "1"
        self._publish_trace = threading.local()
        self._latest_body_state: BodyState | None = None
        self._body_info: BodyInfo | None = None
        self._latest_status: ControlStatus | None = None
        self._acks: dict[tuple[str, ControlOperation], ControlAck] = {}
        self._request_id: str | None = None
        self._request_action: str | None = None
        self._lease_id = ""
        self._external_session_id = 0
        self._next_external_sequence = 1
        # A connection is bound to one immutable robot process as soon as an
        # EndEffectorClient accepts Info. Never transfer an old lease/control
        # context to a restarted robot process.
        self._end_effector_runtime_id: int | None = None
        self._end_effector_runtime_failed = False
        self._cancel_sent = False
        self._closed = False
        self._subscriptions: list[Any] = []

        try:
            if _subscribe_body_state:
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
        return self.enter_control("EXTERNAL", timeout)

    def enter_control(self, action_name: str, timeout: float) -> ControlStatus:
        """Bind this connection to EXTERNAL or the robot's HUMANOID_TELEOP.

        Return only after accepted COMMIT and actual matching RUNNING status.
        timeout bounds entry, not ownership. Release with exit_control().
        HUMANOID_TELEOP accepts EE input; BodyCmd remains EXTERNAL-only.
        """
        if (not isinstance(action_name, str)
                or action_name not in ("EXTERNAL", "HUMANOID_TELEOP")):
            raise ValueError("action_name must be EXTERNAL or HUMANOID_TELEOP")
        deadline = _deadline(timeout)
        with self._condition:
            self._ensure_open()
            if self._request_id is not None:
                raise ExternalControlError("a control request is already active")
            request_id = str(uuid.uuid4())
            self._request_id = request_id
            self._request_action = action_name
            self._lease_id = ""
            self._external_session_id = 0
            self._next_external_sequence = 1
            self._cancel_sent = False
            self._acks.clear()
            prepare = self._request(ControlOperation.PREPARE)

        try:
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
                self._ensure_open()
                if self._request_id != request_id or self._cancel_sent:
                    raise NotRunningError("control entry was cancelled or replaced")
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

            def activate() -> ControlStatus | None:
                # _wait_for calls this under the original condition, including
                # PullController. Never reopen permission after CANCEL/close.
                self._ensure_open()
                if self._request_id != request_id or self._cancel_sent:
                    raise NotRunningError("control entry was cancelled or replaced")
                status = self._owned_running_status_after(status_before_commit)
                if status is not None:
                    self._external_session_id = status.generation
                    self._next_external_sequence = 1
                return status

            return self._wait_for(
                activate,
                deadline,
                f"timed out waiting for owned {action_name}/RUNNING status",
            )
        except BaseException:
            self._best_effort_cancel(request_id=request_id)
            raise

    def publish(self, command: BodyCmd) -> BodyCmd:
        if not self._capture_publish_timing:
            return self._publish(command)
        trace = {'clock': 'monotonic_ns', 'outcome': 'raised'}
        begin = time.perf_counter_ns()
        self._publish_trace.last = trace
        try:
            result = self._publish(command, trace)
            trace['outcome'] = 'returned'
            return result
        except BaseException as error:
            trace['error_type'] = type(error).__name__
            raise
        finally:
            trace['publish_total_ns'] = time.perf_counter_ns() - begin

    @property
    def publish_timing(self):
        """Last publish diagnostics on this thread, or None when disabled."""
        trace = getattr(self._publish_trace, 'last', None)
        return dict(trace) if trace is not None else None

    def _publish(self, command: BodyCmd, trace=None) -> BodyCmd:
        waiting = time.perf_counter_ns() if trace is not None else 0
        with self._condition:
            if trace is not None:
                trace['condition_wait_ns'] = time.perf_counter_ns() - waiting
            self._ensure_open()
            population = None
            if self._body_info is not None and len(self._body_info.joints) == BODY_SLOTS:
                population = tuple(j.populated for j in self._body_info.joints)
            validation = time.perf_counter_ns() if trace is not None else 0
            try:
                _validate_command(command, population)
            finally:
                if trace is not None:
                    trace['validation_ns'] = time.perf_counter_ns() - validation
        preparing = time.perf_counter_ns() if trace is not None else 0
        context = self._reserve_command(for_body=True)
        external_command = replace(
            command,
            protocol_version=BODY_CMD_PROTOCOL_VERSION,
            action=BodyCmdAction.EXTERNAL,
            root_quat=None,
            session_id=context.session_id,
            sequence=context.sequence,
            ts_client_created_us=command.ts_client_created_us or time.monotonic_ns() // 1000,
            ts_client_send_us=time.monotonic_ns() // 1000,
        )
        if trace is not None:
            trace.update(session_id=external_command.session_id,
                         sequence=external_command.sequence,
                         session_replace_ns=time.perf_counter_ns() - preparing)
        sending = time.perf_counter_ns() if trace is not None else 0
        try:
            self.transport.publish(BODY_CMD_TOPIC, external_command)
        finally:
            if trace is not None:
                trace['transport_call_wall_ns'] = time.perf_counter_ns() - sending
                getter = getattr(self.transport, '_last_publish_timing', None)
                if getter is not None:
                    try:
                        trace.update(getter() or {})
                    except Exception as error:
                        trace['timing_error_type'] = type(error).__name__
        return external_command

    def _reserve_command(self, *, for_body: bool = False) -> _CommandContext:
        """Reserve from the connection's stream for a validated BODY/EE call.

        This never enters control or publishes. Failed sends consume their key;
        callers must not replay a reservation after a reconnect or lease change.
        This local snapshot does not replace robot admission.
        Call outside _condition so pull reception cannot invert its lock order.
        """
        with self._condition:
            status = self._require_owned_status()
            if for_body and self._request_action != "EXTERNAL":
                raise NotRunningError("BodyCmd requires an owned EXTERNAL session")
            if status.phase == ExternalPhase.RUNNING:
                if status.generation != self._external_session_id:
                    self._external_session_id = status.generation
                    self._next_external_sequence = 1
            if self._next_external_sequence > _MAX_UINT64:
                raise ExternalControlError("external sequence exhausted")
            context = _CommandContext(
                client_id=self.client_id, lease_id=self._lease_id,
                session_id=self._external_session_id,
                sequence=self._next_external_sequence,
            )
            self._next_external_sequence += 1
            return context

    def _require_owned_status(self) -> ControlStatus:
        """Original shared admission checks; caller holds _condition."""
        self._ensure_open()
        if self._external_session_id == 0 or self._cancel_sent:
            raise NotRunningError("control entry is unconfirmed or cancellation has started")
        status = self._latest_status
        if not self._status_allows_publish(status):
            raise NotRunningError(
                "publish requires the bound action in owned RUNNING or TIMEOUT_HOLD"
            )
        assert status is not None
        if status.phase == ExternalPhase.RUNNING:
            if not _valid_generation(status.generation):
                raise NotRunningError("RUNNING status did not provide a valid generation")
        elif status.generation != self._external_session_id:
            raise NotRunningError(
                "TIMEOUT_HOLD status does not match the learned RUNNING generation"
            )
        return status

    def _control_context(self) -> _ControlContext:
        """No command allocation, generation learning, publish or heartbeat refresh.

        Call outside _condition: PullController drains status before taking the
        shared lock. RPC callers must recheck after blocking and in typed fill.
        """
        with self._condition:
            status = self._require_owned_status()
            return _ControlContext(self.client_id, self._lease_id,
                                   status.generation, self._request_id)

    def _bind_end_effector_runtime(self, robot_runtime_id: int) -> None:
        """Bind this connection to one robot process for all EE clients."""
        if (type(robot_runtime_id) is not int or
                not 0 < robot_runtime_id <= _MAX_UINT64):
            raise ValueError("robot_runtime_id must be a nonzero uint64")
        with self._condition:
            self._ensure_open()
            if self._end_effector_runtime_failed:
                raise ValueError(
                    "robot runtime changed; create a new connection, acquire a "
                    "new control context, and recreate EndEffectorClient"
                )
            if self._end_effector_runtime_id is None:
                self._end_effector_runtime_id = robot_runtime_id
            elif self._end_effector_runtime_id != robot_runtime_id:
                self._end_effector_runtime_failed = True
                raise ValueError(
                    "robot runtime changed; create a new connection, acquire a "
                    "new control context, and recreate EndEffectorClient"
                )

    def exit_external(self, timeout: float) -> ControlStatus:
        """Release the current connection; retained for existing callers."""
        return self.exit_control(timeout)

    def exit_control(self, timeout: float) -> ControlStatus:
        """CANCEL once and wait for the bound action's completed exit."""
        deadline = _deadline(timeout)
        with self._condition:
            self._ensure_open()
            if self._request_id is None:
                raise NotRunningError("no control request is active")
            request_id = self._request_id
            action_name = self._request_action

        self._best_effort_cancel(request_id=request_id)
        cancel_ack = self._wait_for_ack(
            request_id, ControlOperation.CANCEL, deadline
        )
        self._require_accepted(cancel_ack)
        status = self._wait_for(
            lambda: (
                self._latest_status
                if self._latest_status is not None
                and self._latest_status.current_action != action_name
                and self._latest_status.phase
                in (ExternalPhase.IDLE, ExternalPhase.EMERGENCY)
                else None
            ),
            deadline,
            f"timed out waiting for {action_name} to exit",
        )
        with self._condition:
            if self._request_id == request_id:
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
        if not isinstance(lowstate, BodyState) or lowstate.protocol_version != BODY_STATE_PROTOCOL_VERSION:
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
        if self._request_id is None or self._request_action is None:
            raise ExternalControlError("no active request identity")
        return ControlRequest(
            request_id=self._request_id,
            client_id=self.client_id,
            operation=operation,
            action_name=self._request_action,
            lease_id=self._lease_id,
            requested_lease_ms=0,
        )

    def _best_effort_cancel(self, *, request_id: str | None = None) -> None:
        with self._condition:
            if request_id is not None and self._request_id != request_id:
                return
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
            and status.current_action == self._request_action
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
        self._request_action = None
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
    if not isinstance(command.joints, tuple) or len(command.joints) != BODY_SLOTS:
        raise InvalidCommandError("joints must be a 31-slot tuple")
    if command.action != BodyCmdAction.EXTERNAL:
        raise InvalidCommandError("ExternalController requires EXTERNAL action")
    for name in ('input_state_id', 'input_state_ts_real_us', 'ts_client_created_us'):
        value = getattr(command, name)
        if name == 'ts_client_created_us':
            value = value or 0  # _publish fills unset creation times.
        try:
            value = operator.index(value)
        except TypeError as error:
            raise InvalidCommandError(f"{name} must be a uint64") from error
        if not 0 <= value < 2**64:
            raise InvalidCommandError(f"{name} must be a uint64")
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
