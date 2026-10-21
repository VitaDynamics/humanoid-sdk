"""Opt-in keep-last feedback: no Python BodyState/ControlStatus RX callbacks."""
import os
import threading
import time
import uuid
from dataclasses import replace

from .controller import ExternalController, ExternalControlTimeoutError, _deadline, _nonempty_text
from .transport import _AortaTransport, BODY_STATE_TOPIC, CONTROL_STATUS_TOPIC
from .state_arrays import decode_body_state_arrays


class PullTransport(_AortaTransport):
    """Use the pinned Aorta pull ring, preserving the callback transport API.

    Callbacks explicitly registered on this opt-in transport run on the polling
    caller. The original default transport retains asynchronous callbacks.
    """

    def __init__(self, *, node_name, group, publishers=True):
        _nonempty_text(group, 'group')
        _nonempty_text(node_name, 'node_name')
        if not publishers:
            os.environ['AORTA_LOG_TOPIC_ENABLED'] = 'false'
        super().__init__(node_name=node_name, group=group, _publishers=publishers)
        self._pulls = {}
        self._callbacks = {BODY_STATE_TOPIC: [], CONTROL_STATUS_TOPIC: []}
        self._status = None
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
                status = convert(self._bindings.control_status_decoder(sample.payload))
                self._status = status
                for callback in tuple(self._callbacks[CONTROL_STATUS_TOPIC]):
                    callback(status)
            return self._status

    @property
    def control_status(self):
        return self.poll_control_status()

    def wait_state(self, timeout=5.0):
        deadline = _deadline(timeout)
        with self._state_lock:
            self._ensure_open()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('waiting for body state')
            sample = self._pulls[BODY_STATE_TOPIC].recv(remaining)
            received_wall = time.time_ns()  # S1: Python recv return, before decode.
            received_mono = time.monotonic_ns()
            self._ensure_open()
            if sample is None:
                raise TimeoutError('waiting for body state')
            state = replace(decode_body_state_arrays(sample.payload),
                received_monotonic_ns=received_mono, received_realtime_ns=received_wall,
                pull_context_receive_stamp_ns=sample.context.receive_stamp_ns)
            self._ensure_open()
            if self._callbacks[BODY_STATE_TOPIC]:
                dto = state.body_state()
                for callback in tuple(self._callbacks[BODY_STATE_TOPIC]):
                    callback(dto)
            return state

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError('Aorta transport is closed')


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

    def wait_body_state(self, timeout=5.0):
        return self.wait_state(timeout)

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
        return self.transport.wait_state(timeout)

    def wait_body_state(self, timeout=5.0):
        return self.wait_state(timeout).body_state()

    @property
    def status(self):
        self.transport.poll_control_status()
        return super().status

    def publish(self, command):
        self.transport.poll_control_status()
        return super().publish(command)

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
