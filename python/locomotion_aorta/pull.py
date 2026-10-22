"""Opt-in keep-last feedback: no Python BodyState/ControlStatus RX callbacks."""
import struct
import threading
import time
import uuid
from dataclasses import replace

from .controller import (ExternalController, ExternalControlTimeoutError,
                         ControllerClosedError, _deadline, _nonempty_text)
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
                self._status = status
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
            received = self._receive_state(timeout)
            if received is not self._latest_sample or self._latest_state is None:
                sample, wall, mono = received
                state = replace(decode_body_state_arrays(sample.payload),
                    received_monotonic_ns=mono, received_realtime_ns=wall,
                    pull_context_receive_stamp_ns=sample.context.receive_stamp_ns)
                self._ensure_open()
                if received is not self._latest_sample:
                    self._latest_body_state = None
                self._latest_sample = received
                self._latest_state = state
            return self._latest_state

    def _wait_body_state(self, timeout):
        with self._state_lock:
            received = self._receive_state(timeout)
            if received is not self._latest_sample or self._latest_body_state is None:
                sample, _, _ = received
                state = decode_body_state_native(sample.payload)
                self._ensure_open()
                if received is not self._latest_sample:
                    self._latest_state = None
                self._latest_sample = received
                self._latest_body_state = state
            return self._latest_body_state

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

    @property
    def group(self):
        return self._group

    def wait_state(self, timeout=5.0):
        return self.transport.wait_state(timeout)

    @property
    def control_status(self):
        return self.transport.control_status

    def close(self):
        self.transport.close()


class PullController(ExternalController):
    """Original session protocol with pull status pumping on the caller thread."""

    def __init__(self, *, group='default', node_name=None, client_id=None, _transport=None):
        _nonempty_text(group, 'group')
        if client_id:
            _nonempty_text(client_id, 'client_id')
        node_name = node_name or 'locomotion-pull-'+uuid.uuid4().hex
        _nonempty_text(node_name, 'node_name')
        transport = _transport if _transport is not None else PullTransport(
            node_name=node_name, group=group)
        super().__init__(group=group, node_name=node_name, client_id=client_id,
                         _transport=transport, _subscribe_body_state=False)

    def wait_state(self, timeout=5.0):
        with self._condition:
            self._ensure_open()
        return self.transport.wait_state(timeout)

    def wait_body_state(self, timeout=5.0):
        with self._condition:
            self._ensure_open()
        try:
            return self.transport._wait_body_state(timeout)
        except TimeoutError as error:
            raise ExternalControlTimeoutError(str(error)) from error

    @property
    def status(self):
        # The original status property remains readable after close.
        if not self._closed:
            self.transport.poll_control_status()
        return super().status

    def _reserve_command(self):
        with self._condition:
            self._ensure_open()
        self.transport.poll_control_status()
        return super()._reserve_command()

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
