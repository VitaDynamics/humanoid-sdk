"""Independent hand topics and touch control RPC; body EXTERNAL ownership stays separate."""
from collections.abc import Sequence
import math
import threading
import time
import uuid

from . import _hand_codec as codec
from .hand_types import HandCmd, HandInfo, HandJointCmd, HandSide, HandState, HandTouch
from ._ee_pull import FeedbackPulls


HAND_TOPIC_PREFIX = '/sdk/foot_humanoid/hand'
_UINT64_MAX = (1 << 64) - 1


def validate_info(info):
    if not isinstance(info, HandInfo) or info.dof <= 0:
        raise ValueError('valid HandInfo with nonzero dof is required')
    if any(len(values) != info.dof for values in
           (info.joint_names, info.q_min, info.q_max, info.dq_max)):
        raise ValueError('HandInfo per-joint lengths differ from dof')
    if not all(info.joint_names) or len(set(info.joint_names)) != info.dof:
        raise ValueError('HandInfo joint names are invalid')
    for low, high, speed in zip(info.q_min, info.q_max, info.dq_max):
        if not all(math.isfinite(v) for v in (low, high, speed)) or low > high or speed < 0:
            raise ValueError('HandInfo limits are invalid')


class HandClient:
    def __init__(self, side, *, group='default', node=None, read_only=False,
                 on_state=None, on_touch=None, on_info=None, _bindings=None, _session_id=None,
                 pull_feedback=False):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right; hardware side numbers are not accepted')
        self.side = side
        self.wire_side = HandSide.LEFT if side == 'left' else HandSide.RIGHT
        self.topic = f'{HAND_TOPIC_PREFIX}/{side}'
        self.session_id = (_session_id if _session_id is not None
                           else (uuid.uuid4().int & _UINT64_MAX) or 1)
        if type(self.session_id) is not int or not 0 < self.session_id <= _UINT64_MAX:
            raise ValueError('session_id must be nonzero uint64')
        self._bindings = _bindings or codec.load_bindings()
        self._owns_node = node is None
        self.node = node if node is not None else self._bindings.aorta.Node(
            'hand-' + side + '-' + uuid.uuid4().hex, group)
        self._read_only = read_only
        self._condition = threading.Condition()
        self._publish_lock = threading.RLock()
        self._latest = {'state': None, 'touch': None, 'info': None}
        self._callbacks = {'state': on_state, 'touch': on_touch, 'info': on_info}
        self._error = None
        self._closed = False
        self._sequence = 0
        self._publisher = None
        self._subscriptions = []
        self._clients = {}
        self._pulls = FeedbackPulls(self.node, self._receive, self._feedback_error) if pull_feedback else None
        try:
            from ._group import require_node_group
            require_node_group(self.node, group)
            for kind, cls in (('state', 'HandState'), ('touch', 'HandTouch'), ('info', 'HandInfo')):
                decoder = getattr(getattr(self._bindings, cls), cls).GetRootAs
                qos = (self._bindings.aorta.QoS.state_update() if kind == 'info'
                       else self._bindings.aorta.QoS.sensor_data())
                if self._pulls is not None:
                    self._subscriptions.append(self._pulls.add(kind, decoder, self.topic+'/'+kind, qos))
                    continue
                self._subscriptions.append(self.node.create_subscriber_typed(
                    decoder, self.topic + '/' + kind,
                    lambda view, ctx, kind=kind: self._receive(kind, view), qos=qos))
        except BaseException:
            self.close()
            raise

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError('HandClient is closed')
        if self._error is not None:
            raise RuntimeError('invalid hand feedback') from self._error

    def _receive(self, kind, view):
        try:
            value = getattr(codec, 'decode_' + kind)(view)
            if value.side != self.wire_side:
                raise ValueError('hand feedback side differs from topic')
            if kind == 'info':
                validate_info(value)
            with self._condition:
                if self._closed:
                    return
                info = value if kind == 'info' else self._latest['info']
                state = value if kind == 'state' else self._latest['state']
                touch = value if kind == 'touch' else self._latest['touch']
                if info is not None:
                    if state is not None and (state.hand_type != info.hand_type
                                               or len(state.joints) != info.dof):
                        raise ValueError('HandState identity/joint count differs from Info')
                    if touch is not None and (touch.hand_type != info.hand_type
                            or len(touch.fingers) != info.touch_fingers
                            or len(touch.raw_words) != info.touch_fingers * info.touch_words_per_finger):
                        raise ValueError('HandTouch geometry differs from Info')
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
    def touch(self):
        return self._get('touch')

    @property
    def info(self):
        return self._get('info')

    def _wait(self, kind, timeout, after_state_id=None, after_info=None):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
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
                    raise TimeoutError('timed out waiting for hand ' + kind)
                if self._pulls is None:
                    self._condition.wait(remaining)
            if self._pulls is not None:
                self._pulls.poll(kind, remaining)

    def wait_state(self, timeout: float = 5.0, *, after_state_id: int | None = None) -> HandState:
        return self._wait('state', timeout, after_state_id)

    def wait_info(self, timeout: float = 5.0, *, after_info: HandInfo | None = None) -> HandInfo:
        """Wait for info; after_info selects a later topic event, not a timestamp."""
        return self._wait('info', timeout, after_info=after_info)

    def wait_touch(self, timeout: float = 5.0) -> HandTouch:
        return self._wait('touch', timeout)

    def publish(self, q: Sequence[float], dq: Sequence[float] | None = None, *,
                cmd_id: int | None = None, input_state: HandState | None = None) -> HandCmd:
        with self._publish_lock:
            self._ensure_open()
            if self._read_only:
                raise RuntimeError('read-only HandClient cannot publish')
            info = self.info
            validate_info(info)  # do not block the body command loop on a service call
            positions = tuple(q)
            speeds = (0.0,) * info.dof if dq is None else tuple(dq)
            if len(positions) != info.dof or len(speeds) != info.dof:
                raise ValueError('hand command length must equal info.dof')
            for i, (position, speed) in enumerate(zip(positions, speeds)):
                if (isinstance(position, bool) or isinstance(speed, bool)
                        or not math.isfinite(position) or not math.isfinite(speed)
                        or not info.q_min[i] <= position <= info.q_max[i]
                        or not 0 <= speed <= info.dq_max[i]):
                    raise ValueError('hand command is nonfinite or outside info limits')
            state = self.state if input_state is None else input_state
            if state is not None and (state.side != self.wire_side or state.hand_type != info.hand_type):
                raise ValueError('input state belongs to another hand')
            sequence = self._sequence + 1
            if sequence > _UINT64_MAX:
                raise OverflowError('hand sequence exhausted; create a new client/session')
            command_id = sequence & 0xffff if cmd_id is None else cmd_id
            if type(command_id) is not int or not 0 <= command_id <= 0xffff:
                raise ValueError('cmd_id must be uint16')
            created_us = self.node.now() // 1000
            if self._publisher is None:
                self._publisher = self.node.create_publisher_typed(
                    self._bindings.hand_cmd_meta, self.topic + '/cmd',
                    qos=self._bindings.aorta.QoS.realtime_control())
            command = HandCmd(self.session_id, sequence, info.hand_type, self.wire_side,
                              tuple(HandJointCmd(float(q), float(dq)) for q, dq in zip(positions, speeds)),
                              cmd_id=command_id, input_state_id=0 if state is None else state.state_id,
                              input_state_ts_real_us=0 if state is None else state.ts_state_real_us,
                              ts_client_created_us=created_us, ts_client_send_us=self.node.now() // 1000)
            if not all(0 <= ts <= _UINT64_MAX for ts in
                       (command.ts_client_created_us, command.ts_client_send_us)):
                raise ValueError('VitaNow client timestamps must fit uint64')
            self._sequence = sequence  # consume even if transport fails; never replay this sequence
            self._publisher.publish_typed(lambda builder, header:
                                          codec.fill_command(self._bindings, command, builder, header))
            return command

    def _call(self, stem, request, response, timeout, enable=None):
        with self._publish_lock:
            return self._call_locked(stem, request, response, timeout, enable)

    def _call_locked(self, stem, request, response, timeout, enable):
        self._ensure_open()
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        timeout_ms = math.ceil(timeout * 1000)
        key = (stem, timeout_ms)
        if key not in self._clients:
            response_class = getattr(getattr(self._bindings, response), response)
            self._clients[key] = self.node.create_client_typed(
                self.topic + '/' + stem.removeprefix('hand_'), response_class.GetRootAs,
                request_schema_meta=getattr(self._bindings, stem + '_meta'),
                options=self._bindings.aorta.ClientOptions(timeout_ms=timeout_ms))
        client = self._clients[key]
        if not client.raw().wait_for_matching(timeout_ms):
            raise TimeoutError('hand service provider not found: ' + stem)
        reply = client.call(lambda builder, header: codec.fill_request(
            getattr(self._bindings, request), request, builder, header, enable)).decode()
        if reply.Status() != 0:  # aorta.services.common.ServiceStatus.SUCCESS
            raise RuntimeError(f'{stem} failed with ServiceStatus={reply.Status()}')
        return reply

    def set_touch(self, enable: bool, timeout: float = 5.0) -> bool:
        if self._read_only:
            raise RuntimeError('read-only HandClient cannot set_touch')
        if type(enable) is not bool:
            raise ValueError('enable must be boolean')
        reply = self._call('hand_set_touch', 'HandSetTouchRequest', 'HandSetTouchResponse', timeout, enable)
        if reply.Enabled() != enable:
            raise RuntimeError('set_touch reply differs from requested state')
        return reply.Enabled()

    def close(self):
        with self._publish_lock:
            with self._condition:
                if self._closed:
                    return
                self._closed = True
                self._condition.notify_all()
            handles = [*self._subscriptions, *self._clients.values(), self._publisher]
        # Joining subscriber workers must not hold the operation lock: a user
        # callback may still be finishing a publish.
        failure = None
        for handle in handles:
            if handle is not None:
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
        self.close()  # no neutral trajectory and no body lease owned here
