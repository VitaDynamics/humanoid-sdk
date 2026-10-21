"""Private gripper topic lifecycle, identity, cache and teardown."""
from dataclasses import dataclass
import math
import threading
import time
import uuid
from ._ee_pull import FeedbackPulls


GRIPPER_TOPIC_PREFIX = '/sdk/foot_humanoid/gripper'
_UINT64_MAX = (1 << 64) - 1


def _uint64(value, name, *, nonzero=False):
    if type(value) is not int or not int(nonzero) <= value <= _UINT64_MAX:
        raise ValueError(name + ' must fit ' + ('nonzero ' if nonzero else '') + 'uint64')
    return value


@dataclass(frozen=True, slots=True)
class GripperCommandEnvelope:
    """Hand-compatible command identity."""

    session_id: int
    sequence: int
    cmd_id: int
    input_state_id: int
    input_state_ts_real_us: int
    ts_client_created_us: int
    ts_client_send_us: int
    gripper_type: int
    side: int
    protocol_version: int = 1


class GripperClientLifecycle:
    """Manage topic subscriptions and monotonically increasing command identity."""

    def __init__(self, side, *, node, bindings, decode_info, decode_state, group='default',
                 read_only=False, on_info=None, on_state=None,
                 owns_node=False, _session_id=None, pull_feedback=False):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        self.side = side
        self.wire_side = 1 if side == 'left' else 2
        self.topic = f'{GRIPPER_TOPIC_PREFIX}/{side}'
        session = ((uuid.uuid4().int & _UINT64_MAX) or 1) if _session_id is None else _session_id
        self.session_id = _uint64(session, 'session_id', nonzero=True)
        self.node = node
        self._owns_node = owns_node
        self._read_only = read_only
        self._decoders = {'info': decode_info, 'state': decode_state}
        self._callbacks = {'info': on_info, 'state': on_state}
        self._condition = threading.Condition()
        self._publish_lock = threading.RLock()
        self._latest = {'info': None, 'state': None}
        self._subscriptions = []
        self._publisher = None
        self._error = None
        self._closed = False
        self._sequence = 0
        self._pulls = FeedbackPulls(node, self._receive, self._feedback_error) if pull_feedback else None
        try:
            from ._group import require_node_group
            require_node_group(self.node, group)
            for kind, name in (('state', 'GripperState'), ('info', 'GripperInfo')):
                decoder = getattr(getattr(bindings, name), name).GetRootAs
                qos = bindings.aorta.QoS.state_update()
                if self._pulls is not None:
                    self._subscriptions.append(self._pulls.add(kind, decoder, self.topic+'/'+kind, qos))
                    continue
                self._subscriptions.append(self.node.create_subscriber_typed(
                    decoder, self.topic + '/' + kind,
                    lambda view, context, kind=kind: self._receive(kind, view), qos=qos))
        except BaseException:
            self.close()
            raise

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError('GripperClient is closed')
        if self._error is not None:
            raise RuntimeError('invalid gripper feedback') from self._error

    def _receive(self, kind, view):
        try:
            with self._condition:
                if self._closed:
                    return
            value = self._decoders[kind](view)
            if value.side != self.wire_side:
                raise ValueError('gripper feedback side differs from topic')
            with self._condition:
                if self._closed:
                    return
                self._latest[kind] = value
                self._condition.notify_all()
            callback = self._callbacks[kind]
            if callback is not None:
                callback(value)
        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()

    def _get(self, kind):
        if self._pulls is not None:
            with self._condition:
                self._ensure_open()
            self._pulls.poll()
        with self._condition:
            self._ensure_open()
            return self._latest[kind]

    def _feedback_error(self, error):
        with self._condition:
            self._error = error
            self._condition.notify_all()

    @property
    def state(self):
        return self._get('state')

    @property
    def info(self):
        return self._get('info')

    def _wait(self, kind, timeout, *, after_state_id=None, after_info=None):
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        if after_state_id is not None:
            _uint64(after_state_id, 'after_state_id')
        deadline = time.monotonic() + timeout
        while True:
            if self._pulls is not None:
                with self._condition:
                    self._ensure_open()
                self._pulls.poll()
            with self._condition:
                self._ensure_open()
                value = self._latest[kind]
                if (value is not None
                        and (after_state_id is None or value.state_id > after_state_id)
                        and (after_info is None or value is not after_info)):
                    return value
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('timed out waiting for gripper ' + kind)
                if self._pulls is None:
                    self._condition.wait(remaining)
            if self._pulls is not None:
                self._pulls.poll(kind, remaining)

    def wait_info(self, timeout=5.0, *, after_info=None):
        """Use event identity for later periodic info, not its acquisition stamp."""
        return self._wait('info', timeout, after_info=after_info)

    def wait_state(self, timeout=5.0, *, after_state_id=None):
        return self._wait('state', timeout, after_state_id=after_state_id)

    def reserve_envelope(self, *, cmd_id=None, input_state=None):
        """Reserve identity only; final publish must serialize under this lock.

        The client validates the payload before calling this method. Once
        returned, the sequence is consumed even if serialization/transport fails.
        This method does not establish output permission or publish anything.
        """
        with self._publish_lock:
            self._ensure_open()
            if self._read_only:
                raise RuntimeError('read-only GripperClient cannot prepare commands')
            info = self.info
            if info is None:
                raise ValueError('GripperInfo is required before preparing a command')
            state = self.state if input_state is None else input_state
            if state is not None and (state.side != self.wire_side
                                      or state.gripper_type != info.gripper_type):
                raise ValueError('input state belongs to another gripper')
            state_id = 0 if state is None else _uint64(state.state_id, 'input_state_id')
            state_us = 0 if state is None else _uint64(state.ts_state_real_us, 'input_state_ts_real_us')
            sequence = self._sequence + 1
            if sequence > _UINT64_MAX:
                raise OverflowError('gripper sequence exhausted; create a new client/session')
            command_id = sequence & 0xffff if cmd_id is None else cmd_id
            if type(command_id) is not int or not 0 <= command_id <= 0xffff:
                raise ValueError('cmd_id must be uint16')
            created_us = _uint64(self.node.now() // 1000, 'ts_client_created_us')
            send_us = _uint64(self.node.now() // 1000, 'ts_client_send_us')
            self._sequence = sequence
            return GripperCommandEnvelope(
                self.session_id, sequence, command_id, state_id, state_us,
                created_us, send_us, info.gripper_type, self.wire_side)

    def close(self):
        with self._publish_lock:
            with self._condition:
                if self._closed:
                    return
                self._closed = True
                self._condition.notify_all()
            handles = list(self._subscriptions)
            if self._publisher is not None:
                handles.append(self._publisher)
        # A callback can be finishing an operation while a subscription joins.
        failure = None
        for handle in handles:
            try:
                handle.close()
            except Exception as error:
                failure = failure or error
        if self._owns_node:
            try:
                self.node.close()
            except Exception as error:
                failure = failure or error
        if failure is not None:
            raise failure

    def __enter__(self):
        self._ensure_open()
        return self

    def __exit__(self, *_exc):
        self.close()
