"""Opt-in keep-last feedback: no Python BodyState/ControlStatus RX callbacks."""
import struct
import threading
import time
import uuid
from dataclasses import replace

from .controller import (ExternalController, ExternalControlTimeoutError,
                         ControllerClosedError, _deadline, _nonempty_text,
                         _uint64_nonzero, _valid_generation)
from .body_types import BODY_STATE_PROTOCOL_VERSION, ServiceStatus
from .types import ControlStatus, PROTOCOL_VERSION
from .transport import _AortaTransport, BODY_STATE_TOPIC, CONTROL_STATUS_TOPIC
from .state_arrays import decode_body_state_arrays
from ._native_body_codec import decode_body_state_native


class PullTransport(_AortaTransport):
    """Use the pinned Aorta pull ring, preserving the callback transport API.

    Callbacks explicitly registered on this opt-in transport run on the polling
    caller. The original default transport retains asynchronous callbacks.
    """

    def __init__(self, *, node_name, group, publishers=True):
        _nonempty_text(group, 'group')
        _nonempty_text(node_name, 'node_name')
        super().__init__(node_name=node_name, group=group, _publishers=publishers)
        self._pulls = {}
        self._callbacks = {CONTROL_STATUS_TOPIC: []}
        self._status = None
        self.status_decode_errors = 0
        self._latest_sample = None
        self._latest_state = None
        self._latest_body_state = None
        self._state_lock = threading.Lock()
        self._status_lock = threading.RLock()
        self._candidate_gate = None
        try:
            from aorta.options import SubscriberOptions
            for topic, qos in ((BODY_STATE_TOPIC, self._bindings.aorta.QoS.sensor_data()),
                               (CONTROL_STATUS_TOPIC, self._bindings.aorta.QoS.state_update())):
                sub = self.node.create_subscriber_pull(topic, depth=1, qos=qos,
                    options=SubscriberOptions(include_receive_timestamp=True))
                self._pulls[topic] = sub
                self.subscriptions.append(sub)
        except BaseException:
            self.close()
            raise

    def subscribe(self, topic, callback):
        if self._closed:
            raise RuntimeError('Aorta transport is closed')
        if topic == BODY_STATE_TOPIC:
            raise ValueError('use wait_state() for pull BodyState reception')
        if topic not in self._pulls:
            return super().subscribe(topic, callback)
        self._callbacks[topic].append(callback)
        return _PullRegistration(self, topic, callback)

    def poll_control_status(self, timeout=0):
        with self._status_lock:
            self._ensure_open()
            sub = self._pulls[CONTROL_STATUS_TOPIC]
            sample = sub.recv(timeout) if timeout else sub.try_recv()
            self._ensure_open()
            if sample is not None:
                _, convert, _ = self._subscriber_spec(CONTROL_STATUS_TOPIC)
                try:
                    status = convert(self._bindings.control_status_decoder(sample.payload))
                except (ValueError, TypeError, IndexError, AttributeError, struct.error):
                    # Match typed callbacks: retain the last good snapshot. Do
                    # not turn one incompatible phase/frame into CANCEL/PASSIVE.
                    self.status_decode_errors += 1
                    return self._status
                if self._accept_candidate(status, lambda: setattr(self, '_status', status)):
                    for callback in tuple(self._callbacks[CONTROL_STATUS_TOPIC]):
                        callback(status)
            return self._status

    @property
    def control_status(self):
        return self.poll_control_status()

    def wait_state(self, timeout=5.0):
        """Read the latest frame; wait only if no frame has ever been received.

        depth=1 collapses pending samples to the newest. With no pending sample,
        return the previous immutable snapshot with its original receipt clocks.
        timeout=0 is nonblocking. This API does not pace a control/policy loop:
        callers must maintain their configured rate and check state freshness.
        Core receive_stamp_ns is dequeue/assembly time, not network arrival.
        """
        with self._state_lock:
            return self._accept_state(self._receive_state(timeout), arrays=True)

    def _wait_body_state(self, timeout):
        with self._state_lock:
            return self._accept_state(self._receive_state(timeout), arrays=False)

    def _accept_candidate(self, candidate, commit):
        self._ensure_open()
        if self._candidate_gate is None:
            raise ValueError('Pull transport has no bound connection identity')
        return self._candidate_gate(candidate, commit)

    def _accept_state(self, received, *, arrays):
        """Commit only under the owning connection's existing identity gate."""
        # At most the new candidate and the previously accepted sample.
        for _ in range(2):
            cached = self._latest_state if arrays else self._latest_body_state
            if received is self._latest_sample and cached is not None:
                state = cached
            else:
                sample, wall, mono = received
                if arrays:
                    state = replace(decode_body_state_arrays(sample.payload),
                        received_monotonic_ns=mono, received_realtime_ns=wall,
                        pull_context_receive_stamp_ns=sample.context.receive_stamp_ns)
                else:
                    state = decode_body_state_native(sample.payload)

            def commit():
                if received is not self._latest_sample:
                    self._latest_state = self._latest_body_state = None
                self._latest_sample = received
                if arrays:
                    self._latest_state = state
                else:
                    self._latest_body_state = state

            if self._accept_candidate(state, commit):
                return state
            # The candidate has never been committed; retain the old original
            # sample and receipt clocks, including its other representation.
            if self._latest_sample is None or received is self._latest_sample:
                raise ValueError('BodyState does not belong to the discovered runtime')
            received = self._latest_sample
        raise ValueError('BodyState does not belong to the discovered runtime')

    def _receive_state(self, timeout):
        if type(timeout) in (int, float) and timeout == 0:
            deadline = time.monotonic()
        else:
            deadline = _deadline(timeout)
        self._ensure_open()
        sub = self._pulls[BODY_STATE_TOPIC]
        sample = sub.try_recv()
        self._ensure_open()
        if sample is None:
            if self._latest_sample is not None:
                return self._latest_sample
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('waiting for first body state')
            sample = sub.recv(remaining)
        wall, mono = time.time_ns(), time.monotonic_ns()
        self._ensure_open()
        if sample is None:
            raise TimeoutError('waiting for body state')
        return sample, wall, mono

    def _ensure_open(self):
        if self._closed:
            raise ControllerClosedError('Aorta transport is closed')


class _PullRegistration:
    def __init__(self, transport, topic, callback):
        self.transport, self.topic, self.callback = transport, topic, callback
        self.closed = False

    def close(self):
        if not self.closed:
            self.closed = True
            self.transport._callbacks[self.topic].remove(self.callback)


class PullStateSource:
    """Read-only native-array feedback and ControlStatus snapshots; no publishers."""

    def __init__(self, *, group='default', node_name=None, _transport=None):
        self._group = _nonempty_text(group, 'group')
        if node_name is not None:
            _nonempty_text(node_name, 'node_name')
        self.transport = _transport if _transport is not None else PullTransport(
            node_name=node_name or 'body-pull-'+uuid.uuid4().hex,
            group=group, publishers=False)
        self._end_effector_runtime_id = None
        self._end_effector_runtime_lock = threading.Lock()
        self._runtime_changed = False
        self._closed = False
        self._end_effector_clients = set()
        if isinstance(self.transport, PullTransport):
            if self.transport._candidate_gate is not None:
                raise ValueError('Pull transport already has a connection')
            self.transport._candidate_gate = self._commit_pull_candidate

    def _commit_pull_candidate(self, candidate, commit):
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            if isinstance(candidate, ControlStatus):
                valid = (candidate.protocol_version == PROTOCOL_VERSION
                         and _valid_generation(candidate.runtime_id)
                         and candidate.runtime_id == self._end_effector_runtime_id)
            else:
                valid = (candidate.protocol_version == BODY_STATE_PROTOCOL_VERSION
                         and _valid_generation(candidate.runtime_id)
                         and candidate.runtime_id == self._end_effector_runtime_id)
            if not valid:
                return False
            commit()
            return True

    @property
    def group(self):
        return self._group

    def wait_state(self, timeout=5.0):
        state = self.transport.wait_state(timeout)
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            if not _valid_generation(state.runtime_id) or state.runtime_id != self._end_effector_runtime_id:
                raise ValueError('BodyState does not belong to the discovered runtime')
            return state

    def get_robot_info(self, timeout=5.0):
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            observed_runtime = self._end_effector_runtime_id
        reply = self.transport.get_robot_info(timeout)
        handles = []
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            if (reply.status in (ServiceStatus.SUCCESS, ServiceStatus.PARTIAL_SUCCESS)
                    and reply.info is not None and reply.info.body is not None
                    and _valid_generation(reply.runtime_id)):
                runtime = reply.runtime_id
                if (self._end_effector_runtime_id != observed_runtime
                        and self._end_effector_runtime_id != runtime):
                    raise ValueError('discovery context changed while request was in flight')
                if self._end_effector_runtime_id is None:
                    self._end_effector_runtime_id = runtime
                elif self._end_effector_runtime_id != runtime:
                    self._runtime_changed = True
                    for client in tuple(self._end_effector_clients):
                        with client._condition:
                            if not client._closed and client._fatal is None:
                                handles.extend(client._retire_descriptor_locked(
                                    ValueError('robot runtime changed; create a new connection')))
        for handle in handles:
            try:
                handle.close()
            except Exception:
                pass
        if self._runtime_changed:
            raise ValueError('robot runtime changed; create a new connection')
        return reply

    def _ensure_runtime_locked(self):
        if self._closed:
            raise ControllerClosedError('body state source is closed')
        self.transport._ensure_open()
        if self._runtime_changed:
            raise ValueError('robot runtime changed; create a new connection')

    @property
    def control_status(self):
        status = self.transport.control_status
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            if status is None or not _valid_generation(status.runtime_id):
                return None
            return status if status.runtime_id == self._end_effector_runtime_id else None

    def _register_end_effector_client(self, client) -> None:
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            self._end_effector_clients.add(client)

    def _close_end_effector_client(self, client):
        with self._end_effector_runtime_lock:
            self._end_effector_clients.discard(client)
            with client._condition:
                return client._close_locked()

    def _bind_end_effector_runtime(self, client, robot_runtime_id: int) -> bool:
        if type(robot_runtime_id) is not int or not 0 < robot_runtime_id < 2**64:
            raise ValueError('robot_runtime_id must be a nonzero uint64')
        with self._end_effector_runtime_lock:
            self._ensure_runtime_locked()
            with client._condition:
                client._ensure_open()
                return self._end_effector_runtime_id == robot_runtime_id

    def close(self):
        with self._end_effector_runtime_lock:
            if self._closed:
                return
            self._closed = True
            handles = []
            for client in tuple(self._end_effector_clients):
                with client._condition:
                    handles.extend(client._close_locked())
            self._end_effector_clients.clear()
        for handle in handles:
            try:
                handle.close()
            except Exception:
                pass
        self.transport.close()


class PullController(ExternalController):
    """Original session protocol with pull status pumping on the caller thread."""

    def __init__(self, *, group='default', node_name=None, client_id=None, _transport=None):
        _nonempty_text(group, 'group')
        node_name = node_name or 'locomotion-pull-'+uuid.uuid4().hex
        _nonempty_text(node_name, 'node_name')
        if client_id is not None:
            _uint64_nonzero(client_id, 'client_id')
        transport = _transport if _transport is not None else PullTransport(
            node_name=node_name, group=group)
        if isinstance(transport, PullTransport) and transport._candidate_gate is not None:
            raise ValueError('Pull transport already has a connection')
        super().__init__(group=group, node_name=node_name, client_id=client_id,
                         _transport=transport, _subscribe_body_state=False)
        if isinstance(transport, PullTransport):
            transport._candidate_gate = self._commit_pull_candidate

    def _commit_pull_candidate(self, candidate, commit):
        with self._condition:
            self._ensure_open()
            self.transport._ensure_open()
            if isinstance(candidate, ControlStatus):
                valid = (candidate.protocol_version == PROTOCOL_VERSION
                         and _valid_generation(candidate.runtime_id)
                         and candidate.runtime_id == self._end_effector_runtime_id)
            else:
                valid = (candidate.protocol_version == BODY_STATE_PROTOCOL_VERSION
                         and _valid_generation(candidate.runtime_id)
                         and candidate.runtime_id == self._end_effector_runtime_id)
            if not valid:
                return False
            commit()
            return True

    def wait_state(self, timeout=5.0):
        with self._condition:
            self._ensure_open()
        state = self.transport.wait_state(timeout)
        with self._condition:
            self._ensure_open()
            self.transport._ensure_open()
            if not _valid_generation(state.runtime_id) or state.runtime_id != self._end_effector_runtime_id:
                raise ValueError('BodyState does not belong to the discovered runtime')
            return state

    def wait_body_state(self, timeout=5.0):
        with self._condition:
            self._ensure_open()
        try:
            state = self.transport._wait_body_state(timeout)
            with self._condition:
                self._ensure_open()
                self.transport._ensure_open()
                if not _valid_generation(state.runtime_id) or state.runtime_id != self._end_effector_runtime_id:
                    raise ValueError('BodyState does not belong to the discovered runtime')
                return state
        except TimeoutError as error:
            raise ExternalControlTimeoutError(str(error)) from error

    @property
    def status(self):
        # The original status property remains readable after close.
        if not self._closed:
            self.transport.poll_control_status()
        return super().status

    def _reserve_command(self, *, for_body=False, expected_runtime_id=None):
        with self._condition:
            self._ensure_open()
        self.transport.poll_control_status()
        return super()._reserve_command(for_body=for_body, expected_runtime_id=expected_runtime_id)

    def _reserve_end_effector_command(self, client, robot_runtime_id, prepare):
        with self._condition:
            self._ensure_open()
        self.transport.poll_control_status()
        return super()._reserve_end_effector_command(
            client, robot_runtime_id, prepare
        )

    def _control_context(self):
        with self._condition:
            self._ensure_open()
        self.transport.poll_control_status()
        return super()._control_context()

    def _end_effector_control_context(self, client, robot_runtime_id, operation):
        with self._condition:
            self._ensure_open()
        self.transport.poll_control_status()
        return super()._end_effector_control_context(
            client, robot_runtime_id, operation
        )

    def _wait_for(self, value, deadline, timeout_message):
        while True:
            with self._condition:
                self._ensure_open()
            self.transport.poll_control_status()
            with self._condition:
                self._ensure_open()
                result = value()
                if result is not None:
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ExternalControlTimeoutError(timeout_message)
                # The previous example's 50Hz polling cadence; protocol deadlines
                # and ACK notification remain the original controller's.
                self._condition.wait(min(remaining, .02))
