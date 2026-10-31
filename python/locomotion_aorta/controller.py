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
    _positive_timeout,
)

from .body_types import (
    BODY_SLOTS, BODY_CMD_PROTOCOL_VERSION, BODY_STATE_PROTOCOL_VERSION, BodyState, BodyCmd, BodyJointCmd, BodyCmdAction,
    BodyInfo, GetRobotInfoResponse, ServiceStatus,
    _BODY_FIELD_ALL, _BODY_FIELD_KP, _BODY_FIELD_Q,
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

    client_id: int
    control_id: int
    session_id: int
    sequence: int
    runtime_id: int
    request_id: str


@dataclass(frozen=True, slots=True)
class _ControlContext:
    """Local RPC authorization snapshot; generation/request are never wire credentials."""

    client_id: int
    control_id: int
    generation: int
    request_id: str


@dataclass(slots=True)
class _RuntimeBinding:
    """One connection's startup binding; caller owns its existing lock and I/O."""

    runtime_id: int | None = None
    poisoned: bool = False

    def accept_discovery(self, reply, observed_runtime) -> bool:
        if (reply.status not in (ServiceStatus.SUCCESS, ServiceStatus.PARTIAL_SUCCESS)
                or reply.info is None or reply.info.body is None
                or not _valid_generation(reply.runtime_id)):
            return False
        runtime = reply.runtime_id
        if self.runtime_id != observed_runtime and self.runtime_id != runtime:
            raise ValueError('discovery context changed while request was in flight')
        if self.runtime_id is None:
            self.runtime_id = runtime
        elif self.runtime_id != runtime:
            self.retire()
        return not self.poisoned

    def retire(self) -> None:
        # Keep the creating R: retirement never rebinds existing objects.
        self.poisoned = True


def _matches_control_context(current, request_id, client_id, control_id, generation):
    return not (current.request_id != request_id or current.client_id != client_id
                or current.control_id != control_id or current.generation != generation)


def _close_handles(handles):
    for handle in handles:
        try:
            handle.close()
        except Exception:
            pass


def _retire_ee_handles(clients, failure):
    """Collect under the caller's connection lock; never close I/O here."""
    handles = []
    for client in tuple(clients):
        with client._condition:
            if not client._closed and client._fatal is None:
                handles.extend(client._retire_descriptor_locked(failure))
    return handles


def _close_ee_handles(clients):
    handles = []
    for client in tuple(clients):
        with client._condition:
            handles.extend(client._close_locked())
    clients.clear()
    return handles


def _detach_ee_handles(clients, client):
    clients.discard(client)
    with client._condition:
        return client._close_locked()


class ExternalController:
    def __init__(
        self,
        *,
        group: str = "default",
        node_name: str | None = None,
        client_id: int | None = None,
        _transport: Transport | None = None,
        _subscribe_body_state: bool = True,
    ) -> None:
        self.group = _nonempty_text(group, "group")
        self.node_name = _nonempty_text(
            node_name or f"locomotion-aorta-{uuid.uuid4()}", "node_name"
        )
        self._proposed_client_id = _uint64_nonzero(
            client_id if client_id is not None else ((uuid.uuid4().int & _MAX_UINT64) or 1),
            'client_id')
        self.client_id = 0  # learned only from the correlated PREPARE ACK
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
        self._expected_ack: ControlOperation | None = None
        self._request_id: str | None = None
        self._request_action: str | None = None
        self._control_id = 0
        self._external_session_id = 0
        self._next_external_sequence = 1
        # The discovery response binds the connection, not an EE topic.
        self._runtime_binding = _RuntimeBinding()
        self._end_effector_clients: set[Any] = set()
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
            observed_runtime = self._runtime_binding.runtime_id
        reply = self.transport.get_robot_info(timeout)
        handles = []
        with self._condition:
            self._ensure_open()
            if self._runtime_binding.accept_discovery(reply, observed_runtime):
                self._body_info = reply.info.body
            elif self._runtime_binding.poisoned:
                handles = self._retire_runtime_locked()
        _close_handles(handles)
        if self._runtime_binding.poisoned:
            raise self._end_effector_runtime_changed()
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
            if self._runtime_binding.runtime_id is None:
                raise NotRunningError('get_robot_info must establish a valid runtime first')
            runtime_id = self._runtime_binding.runtime_id
        if action_name == 'EXTERNAL':
            self.transport._prepare_body_output()
        with self._condition:
            self._ensure_open()
            if self._request_id is not None:
                raise ExternalControlError('a control request is already active')
            if self._runtime_binding.runtime_id != runtime_id:
                raise self._end_effector_runtime_changed()
            if time.monotonic() >= deadline:
                raise ExternalControlTimeoutError('timed out preparing control output')
            request_id = str(uuid.uuid4())
            self._request_id = request_id
            self._request_action = action_name
            self.client_id = 0
            self._control_id = 0
            self._external_session_id = 0
            self._next_external_sequence = 1
            self._cancel_sent = False
            self._acks.clear()
            prepare = self._request(ControlOperation.PREPARE)
            self._expected_ack = ControlOperation.PREPARE

        try:
            self._publish_control(prepare)
            prepare_ack = self._wait_for_ack(
                request_id, ControlOperation.PREPARE, deadline
            )
            self._require_accepted(prepare_ack)
            if not _valid_generation(prepare_ack.control_id) or not _valid_generation(prepare_ack.client_id):
                raise ControlRejectedError(
                    "PREPARE was accepted without canonical client/control identity"
                )
            with self._condition:
                self._ensure_open()
                if self._request_id != request_id or self._cancel_sent:
                    raise NotRunningError("control entry was cancelled or replaced")
                self.client_id = prepare_ack.client_id
                self._control_id = prepare_ack.control_id
                status_before_commit = self._latest_status
                commit = self._request(ControlOperation.COMMIT)
                self._expected_ack = ControlOperation.COMMIT

            self._publish_control(commit)
            commit_ack = self._wait_for_ack(
                request_id, ControlOperation.COMMIT, deadline
            )
            self._require_accepted(commit_ack)
            if commit_ack.control_id != self._control_id or commit_ack.client_id != self.client_id:
                raise ControlRejectedError(
                    "COMMIT acknowledgement changed the prepared identity"
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

    def publish(self, command: BodyCmd, *, target_names=None, required_fields=None) -> BodyCmd:
        if not self._capture_publish_timing:
            return self._publish(command, target_names=target_names, required_fields=required_fields)
        trace = {'clock': 'monotonic_ns', 'outcome': 'raised'}
        begin = time.perf_counter_ns()
        self._publish_trace.last = trace
        try:
            result = self._publish(command, trace, target_names=target_names, required_fields=required_fields)
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

    def _publish(self, command: BodyCmd, trace=None, *, target_names=None, required_fields=None) -> BodyCmd:
        waiting = time.perf_counter_ns() if trace is not None else 0
        with self._condition:
            if trace is not None:
                trace['condition_wait_ns'] = time.perf_counter_ns() - waiting
            self._ensure_open()
            validation = time.perf_counter_ns() if trace is not None else 0
            try:
                _validate_command(command)
                command_runtime_id = command.runtime_id
                if (command_runtime_id is not None and
                        (not _valid_generation(command_runtime_id) or
                         command_runtime_id != self._runtime_binding.runtime_id)):
                    raise InvalidCommandError('BodyCmd belongs to a different robot runtime')
            finally:
                if trace is not None:
                    trace['validation_ns'] = time.perf_counter_ns() - validation
        preparing = time.perf_counter_ns() if trace is not None else 0
        # PullController's _control_context pumps status outside _condition.
        current = self._control_context()
        with self._condition:
            self._ensure_open()
            body_info = self._body_info
            runtime_id = self._runtime_binding.runtime_id
            _validate_body_targets(command, body_info, target_names, required_fields)
            context = self._reserve_command_locked(
                for_body=True, expected_runtime_id=command_runtime_id,
                expected_control=current, expected_body_info=body_info,
                expected_discovery_runtime=runtime_id)
        external_command = replace(
            command,
            protocol_version=BODY_CMD_PROTOCOL_VERSION,
            action=BodyCmdAction.EXTERNAL,
            root_quat=(1.0, 0.0, 0.0, 0.0),
            runtime_id=context.runtime_id,
            control_id=context.control_id,
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
            def guard(submit):
                # PullController pumps its status ring here, outside _condition.
                self._poll_status_if_open()
                with self._condition:
                    current = self._owned_control_context_locked()
                    if (self._runtime_binding.runtime_id != context.runtime_id
                            or self._body_info is not body_info
                            or not _matches_control_context(current, context.request_id,
                                context.client_id, context.control_id, context.session_id)
                            or self._external_session_id != context.session_id
                            or (command_runtime_id is not None and
                                command_runtime_id != context.runtime_id)
                            or self._request_action != 'EXTERNAL'):
                        raise NotRunningError('Body control context changed before submission')
                    return submit()
            self.transport._publish_body(external_command, guard)
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

    def _reserve_command(self, *, for_body: bool = False,
                         expected_runtime_id: int | None = None) -> _CommandContext:
        """Reserve from the connection's stream for a validated BODY/EE call.

        This never enters control or publishes. Failed sends consume their key;
        callers must not replay a reservation after a reconnect or lease change.
        This local snapshot does not replace robot admission.
        Call outside _condition so pull reception cannot invert its lock order.
        """
        with self._condition:
            return self._reserve_command_locked(for_body=for_body, expected_runtime_id=expected_runtime_id)

    def _reserve_command_locked(self, *, for_body: bool = False,
                                expected_runtime_id: int | None = None,
                                expected_control: _ControlContext | None = None,
                                expected_body_info: BodyInfo | None = None,
                                expected_discovery_runtime: int | None = None) -> _CommandContext:
        """Reserve one key while the caller holds the connection condition."""
        status = self._require_owned_status()
        if (expected_control is not None and
                (self._body_info is not expected_body_info or
                 self._runtime_binding.runtime_id != expected_discovery_runtime or
                 not _matches_control_context(expected_control, self._request_id,
                     self.client_id, self._control_id, status.generation))):
            raise NotRunningError('Body context changed during target validation')
        if for_body and self._request_action != "EXTERNAL":
            raise NotRunningError("BodyCmd requires an owned EXTERNAL session")
        if expected_runtime_id is not None and (
                not _valid_generation(expected_runtime_id) or
                expected_runtime_id != self._runtime_binding.runtime_id):
            raise InvalidCommandError('BodyCmd belongs to a different robot runtime')
        if status.phase == ExternalPhase.RUNNING:
            if status.generation != self._external_session_id:
                self._external_session_id = status.generation
                self._next_external_sequence = 1
        if self._next_external_sequence > _MAX_UINT64:
            raise ExternalControlError("external sequence exhausted")
        context = _CommandContext(
            client_id=self.client_id, control_id=self._control_id,
            session_id=self._external_session_id,
            sequence=self._next_external_sequence,
            runtime_id=self._runtime_binding.runtime_id,
            request_id=self._request_id,
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

    def _poll_status_if_open(self) -> None:
        """Callback transport has no status ring; Pull overrides this outside locks."""

    def _owned_control_context_locked(self) -> _ControlContext:
        status = self._require_owned_status()
        return _ControlContext(self.client_id, self._control_id,
                               status.generation, self._request_id)

    def _control_context(self) -> _ControlContext:
        """No command allocation, generation learning, publish or heartbeat refresh.

        Call outside _condition: PullController drains status before taking the
        shared lock. RPC callers must recheck after blocking and in typed fill.
        """
        with self._condition:
            return self._owned_control_context_locked()

    @staticmethod
    def _end_effector_runtime_changed() -> ValueError:
        return ValueError(
            "robot runtime changed; create a new connection, acquire a "
            "new control context, and recreate EndEffectorClient"
        )

    def _retire_runtime_locked(self) -> list[Any]:
        """Retire the existing connection after an authoritative runtime change."""
        failure = self._end_effector_runtime_changed()
        self._runtime_binding.retire()
        self._latest_body_state = None
        self._latest_status = None
        self._cancel_sent = True
        handles = _retire_ee_handles(self._end_effector_clients, failure)
        self._condition.notify_all()
        return handles

    def _register_end_effector_client(self, client: Any) -> None:
        """Attach one EE client to this connection's existing lifecycle."""
        with self._condition:
            self._ensure_open()
            self._end_effector_clients.add(client)

    def _close_end_effector_client(self, client: Any) -> list[Any]:
        """Detach and retire one client; returned handles close without locks."""
        with self._condition:
            return _detach_ee_handles(self._end_effector_clients, client)

    def _require_end_effector_runtime_locked(
        self, client: Any, robot_runtime_id: int
    ) -> None:
        """Check one live registered EE client while both conditions are held."""
        self._ensure_open()
        client._ensure_open()
        if client not in self._end_effector_clients:
            raise ControllerClosedError("end-effector client is detached")
        if self._runtime_binding.runtime_id != robot_runtime_id:
            raise self._end_effector_runtime_changed()

    def _bind_end_effector_runtime(self, client: Any, robot_runtime_id: int) -> bool:
        """Accept a topic only when it matches the discovered process."""
        _uint64_nonzero(robot_runtime_id, 'robot_runtime_id')
        with self._condition:
            self._ensure_open()
            with client._condition:
                client._ensure_open()
                if client not in self._end_effector_clients:
                    raise ControllerClosedError("end-effector client is detached")
            return self._runtime_binding.runtime_id == robot_runtime_id

    def _run_end_effector_gate(
        self, client: Any, robot_runtime_id: int, operation: Callable[[], _T]
    ) -> _T:
        """Linearize an EE-only operation with shared runtime invalidation."""
        with self._condition:
            with client._condition:
                self._require_end_effector_runtime_locked(client, robot_runtime_id)
                return operation()

    def _reserve_end_effector_command(
        self, client: Any, robot_runtime_id: int, prepare: Callable[[], _T]
    ) -> tuple[_CommandContext, _T]:
        """Validate/prepare an EE send, then allocate under the runtime gate."""
        with self._condition:
            with client._condition:
                self._require_end_effector_runtime_locked(client, robot_runtime_id)
                prepared = prepare()
                return self._reserve_command_locked(), prepared

    def _end_effector_control_context(
        self, client: Any, robot_runtime_id: int, operation: Callable[[_ControlContext], _T]
    ) -> _T:
        """Run one EE RPC check/fill under the runtime and control gates."""
        with self._condition:
            with client._condition:
                self._require_end_effector_runtime_locked(client, robot_runtime_id)
                return operation(self._owned_control_context_locked())

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
            should_cancel = (self._request_id is not None and not self._cancel_sent
                             and not self._runtime_binding.poisoned)
            if should_cancel:
                self._cancel_sent = True
                cancel = self._request(ControlOperation.CANCEL)
                self._expected_ack = ControlOperation.CANCEL
            else:
                cancel = None
            handles = _close_ee_handles(self._end_effector_clients)
            self._condition.notify_all()

        if cancel is not None:
            try:
                self._publish_control(cancel)
            except BaseException:
                pass
        _close_handles(handles)
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
        if (not isinstance(lowstate, BodyState) or
                lowstate.protocol_version != BODY_STATE_PROTOCOL_VERSION or
                not isinstance(lowstate.joints, tuple) or len(lowstate.joints) != BODY_SLOTS):
            return
        with self._condition:
            if (not self._closed and not self._runtime_binding.poisoned
                    and _valid_generation(lowstate.runtime_id)
                    and lowstate.runtime_id == self._runtime_binding.runtime_id):
                self._latest_body_state = lowstate
                self._condition.notify_all()

    def _on_ack(self, ack: Any) -> None:
        if (
            not isinstance(ack, ControlAck)
            or ack.protocol_version != PROTOCOL_VERSION
            or not _valid_uint64(ack.client_id)
            or not _valid_uint64(ack.control_id)
        ):
            return
        handles = []
        with self._condition:
            if not self._closed and ack.request_id == self._request_id:
                if ack.operation != self._expected_ack:
                    return
                if self._cancel_sent and ack.operation != ControlOperation.CANCEL:
                    return
                if ack.operation not in (ControlOperation.PREPARE, ControlOperation.COMMIT, ControlOperation.CANCEL):
                    return
                if (ack.operation != ControlOperation.PREPARE and self._control_id
                        and (ack.client_id != self.client_id or ack.control_id != self._control_id)):
                    return
                if not _valid_generation(ack.runtime_id):
                    return
                if ack.runtime_id != self._runtime_binding.runtime_id:
                    handles = self._retire_runtime_locked()
                else:
                    key = (ack.request_id, ack.operation)
                    if key not in self._acks:
                        self._acks[key] = ack
                        self._condition.notify_all()
        _close_handles(handles)

    def _on_status(self, status: Any) -> None:
        if (
            not isinstance(status, ControlStatus)
            or status.protocol_version != PROTOCOL_VERSION
            or not _valid_uint64(status.client_id)
            or not _valid_uint64(status.control_id)
        ):
            return
        with self._condition:
            if (not self._closed and not self._runtime_binding.poisoned
                    and _valid_generation(status.runtime_id)
                    and status.runtime_id == self._runtime_binding.runtime_id):
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
            client_id=self.client_id or self._proposed_client_id,
            operation=operation,
            action_name=self._request_action,
            control_id=self._control_id,
            runtime_id=self._runtime_binding.runtime_id,
        )

    def _best_effort_cancel(self, *, request_id: str | None = None) -> None:
        with self._condition:
            if request_id is not None and self._request_id != request_id:
                return
            if self._request_id is None or self._cancel_sent:
                return
            self._cancel_sent = True
            cancel = self._request(ControlOperation.CANCEL)
            self._expected_ack = ControlOperation.CANCEL
        try:
            self._publish_control(cancel)
        except BaseException:
            pass

    def _publish_control(self, request: ControlRequest) -> None:
        def guard(fill):
            with self._condition:
                if self._runtime_binding.poisoned:
                    raise self._end_effector_runtime_changed()
                if request.request_id != self._request_id or request.runtime_id != self._runtime_binding.runtime_id:
                    raise NotRunningError('control request context changed before encoding')
                if request.operation != self._expected_ack:
                    raise NotRunningError('control request phase changed before encoding')
                if request.operation == ControlOperation.CANCEL:
                    if not self._cancel_sent:
                        raise NotRunningError('cancellation is no longer active')
                elif self._closed or self._cancel_sent:
                    raise NotRunningError('control request was cancelled or closed')
                if request.control_id != self._control_id:
                    raise NotRunningError('control identity changed before encoding')
                if request.client_id != (self.client_id or self._proposed_client_id):
                    raise NotRunningError('client identity changed before encoding')
                return fill()
        self.transport._publish_control(request, guard)

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
            and self._control_id
            and status is not None
            and status.current_action == self._request_action
            and status.phase in (ExternalPhase.RUNNING, ExternalPhase.TIMEOUT_HOLD)
            and status.client_id == self.client_id
            and status.control_id == self._control_id
            and status.runtime_id == self._runtime_binding.runtime_id
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
        self.client_id = 0
        self._control_id = 0
        self._external_session_id = 0
        self._next_external_sequence = 1
        self._cancel_sent = False
        self._expected_ack = None

    def _ensure_open(self) -> None:
        if self._closed:
            raise ControllerClosedError("controller is closed")
        if self._runtime_binding.poisoned:
            raise self._end_effector_runtime_changed()


def _uint64_nonzero(value: int, name: str) -> int:
    if not _valid_uint64(value) or value == 0:
        raise ValueError(f'{name} must be a nonzero uint64')
    return value


def _valid_uint64(value: int) -> bool:
    return type(value) is int and 0 <= value <= _MAX_UINT64


def _nonempty_text(value: str, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _deadline(timeout: float) -> float:
    _positive_timeout(timeout)
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


def _validate_body_targets(command: BodyCmd, body_info: BodyInfo | None,
                           target_names, required_fields) -> None:
    if (body_info is None or body_info.slot_count != BODY_SLOTS
            or len(body_info.joint_names) != BODY_SLOTS or len(body_info.joints) != BODY_SLOTS
            or len(set(body_info.joint_names)) != BODY_SLOTS
            or any(cap.body_index != i for i, cap in enumerate(body_info.joints))):
        raise InvalidCommandError('validated BodyInfo is required before publish')
    if (not isinstance(target_names, tuple) or not target_names
            or any(not isinstance(name, str) or not name for name in target_names)
            or len(set(target_names)) != len(target_names)):
        raise InvalidCommandError('BodyCmd target scope must be explicitly declared')
    if type(required_fields) is not int or not 0 < required_fields <= _BODY_FIELD_ALL:
        raise InvalidCommandError('BodyCmd required fields must be explicitly declared')
    if required_fields & _BODY_FIELD_KP and not required_fields & _BODY_FIELD_Q:
        raise InvalidCommandError('position gain requires the declared position field')
    roster = {name: i for i, name in enumerate(body_info.joint_names)}
    if any(name not in roster for name in target_names):
        raise InvalidCommandError('BodyCmd target is not in the confirmed roster')
    slots = frozenset(roster[name] for name in target_names)
    unavailable = [body_info.joint_names[i] for i in slots if body_info.joints[i].is_mock]
    if unavailable:
        raise InvalidCommandError('BodyCmd targets unavailable: ' + ', '.join(unavailable))
    fields = ('q', 'dq', 'tau', 'kp', 'kd')
    for i, joint in enumerate(command.joints):
        if i not in slots:
            if any(getattr(joint, name) != 0.0 for name in fields):
                raise InvalidCommandError(f'BodyCmd non-target padding is not neutral: {i}')
            continue
        cap = body_info.joints[i]
        if cap.supported_fields & required_fields != required_fields:
            raise InvalidCommandError(f'BodyCmd target capability unavailable: {i}')
        if required_fields & _BODY_FIELD_Q and not cap.valid_limits & _BODY_FIELD_Q:
            raise InvalidCommandError(f'BodyCmd target q limit unavailable: {i}')
        # A missing non-Q range is unknown; native admission remains authoritative.
        for bit, name in enumerate(fields):
            value = getattr(joint, name)
            field_bit = 1 << bit
            if cap.valid_limits & field_bit:
                if not getattr(cap, name + '_min') <= value <= getattr(cap, name + '_max'):
                    raise InvalidCommandError(f'BodyCmd target {name} outside limit: {i}')
            if not required_fields & field_bit and value != 0.0:
                raise InvalidCommandError(f'BodyCmd undeclared field is not neutral: {i}.{name}')


def _validate_command(command: BodyCmd) -> None:
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
        for field in ("q", "dq", "tau", "kp", "kd"):
            value = getattr(motor, field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise InvalidCommandError(f"joints[{index}].{field} must be numeric")
            if not math.isfinite(value):
                raise InvalidCommandError(f"joints[{index}].{field} must be finite")
