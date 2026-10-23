"""New EE development client on one existing EXTERNAL connection.

The current source candidate supports K=0. It does not implement set_touch or
Body3 observation binding and is not a released replacement for the old API.
"""
import struct
import threading
import time
from dataclasses import fields, is_dataclass, replace

from . import _end_effector_codec as codec
from .controller import ControllerClosedError, ExternalController, InvalidCommandError, _deadline
from .pull import PullStateSource
from .end_effector_types import (
    CommandKey, ControlRef, DescriptorRef, EndEffectorSide, EndEffectorSubmission,
    SampleRef, SampleStream,
)


def _token(value):
    """Compare immutable description data, including invalid/NaN float32 bits."""
    if is_dataclass(value):
        return tuple(_token(getattr(value, f.name)) for f in fields(value))
    if isinstance(value, float):
        return struct.pack('<f', value)
    if isinstance(value, tuple):
        return tuple(_token(item) for item in value)
    return value


def _description(info):
    return _token(replace(info, ts_state_pub_us=0, time_valid_mask=info.time_valid_mask & ~2))


def _layout(info):
    n = len(info.axis_names)
    command_shape = tuple(n if any(mask & bit for mask in info.supported_command_fields) else 0
                          for _, bit in codec._COMMAND_COLUMNS)
    state_shape = tuple(n if any(mask & bit for mask in info.supported_state_fields) else 0
                        for _, bit in codec._STATE_COLUMNS)
    return (info.layout_id, info.side, info.axis_names, info.position_domain,
            info.torque_domain, tuple(len(getattr(info, name)) for name in codec._INFO_VECTORS),
            command_shape, state_shape,
            info.touch_available, _token(info.touch_regions))


class EndEffectorClient:
    def __init__(self, side, *, connection, read_only=False,
                 on_info=None, on_state=None, on_touch=None):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if type(read_only) is not bool:
            raise ValueError('read_only must be boolean')
        if not isinstance(connection, ExternalController) and not (
                read_only and isinstance(connection, PullStateSource)):
            raise TypeError('writes require a shared controller; a read-only client may borrow PullStateSource')
        for callback in (on_info, on_state, on_touch):
            if callback is not None and not callable(callback):
                raise TypeError('callbacks must be callable')
        if isinstance(connection, ExternalController):
            with connection._condition:
                connection._ensure_open()
        self.side = side
        self.connection = connection
        self._side = EndEffectorSide.LEFT if side == 'left' else EndEffectorSide.RIGHT
        self._read_only = read_only
        self._bindings = codec.load_bindings()
        self._node = connection.transport.node
        self._topic = '/sdk/foot_humanoid/end_effector/' + side
        self._condition = threading.Condition()
        self._publish_lock = threading.RLock()
        self._closed = False
        self._fatal = None
        self._latest = {'info': None, 'state': None, 'touch': None}
        self._errors = {'info': None, 'state': None, 'touch': None}
        self._callbacks = {'info': on_info, 'state': on_state, 'touch': on_touch}
        self._subscriptions = {}
        self._registering = set()
        self._publisher = None
        self._raw_shape = None
        try:
            self._register('info')
            self._register('state')
            with self._condition:
                self._ensure_open()
        except BaseException:
            self.close()
            raise

    def _ensure_open(self):
        if self._closed:
            raise ControllerClosedError('end-effector client is closed')
        if self._fatal is not None:
            raise ValueError('EE descriptor lifecycle changed; recreate the client on a refreshed connection') from self._fatal

    def _register(self, kind):
        with self._condition:
            if self._closed or kind in self._subscriptions or kind in self._registering:
                return
            self._registering.add(kind)
        try:
            module = getattr(self._bindings, 'EndEffector' + kind.capitalize())
            decoder = getattr(module, 'EndEffector' + kind.capitalize()).GetRootAs
            qos = (self._bindings.aorta.QoS.state_update() if kind == 'info'
                   else self._bindings.aorta.QoS.sensor_data())
            handle = self._node.create_subscriber_typed(
                decoder, self._topic + '/' + kind,
                lambda view, context: self._receive(kind, view), qos=qos)
        except BaseException:
            with self._condition:
                self._registering.discard(kind)
                self._condition.notify_all()
            raise
        with self._condition:
            self._registering.discard(kind)
            discard = self._closed
            if not discard:
                self._subscriptions[kind] = handle
        # A registration started before close may finish later. Never attach it
        # to the closed object, and never wait for a callback while holding locks.
        if discard:
            handle.close()

    def _receive(self, kind, view):
        with self._condition:
            if self._closed or self._fatal is not None:
                return
        try:
            value = getattr(codec, 'decode_' + kind)(view)
            with self._publish_lock, self._condition:
                if self._closed or self._fatal is not None:
                    return
                info = self._latest['info']
                if kind == 'info':
                    codec.validate_info(value)
                    if value.side != self._side:
                        raise ValueError('Info side differs from endpoint')
                    if info is not None:
                        old, new = info.descriptor_ref, value.descriptor_ref
                        if new.robot_runtime_id != old.robot_runtime_id:
                            self._fatal = ValueError('robot runtime changed')
                            self._condition.notify_all()
                            return
                        if new.descriptor_epoch < old.descriptor_epoch:
                            return
                        if new == old and _description(value) != _description(info):
                            self._fatal = ValueError('immutable Info changed without a new DescriptorRef')
                            self._condition.notify_all()
                            return
                        if new != old:
                            if _layout(value) != _layout(info):
                                self._fatal = ValueError('layout/shape changed; no hot resize')
                                self._condition.notify_all()
                                return
                            self._latest['state'] = self._latest['touch'] = None
                            self._errors['state'] = self._errors['touch'] = None
                            self._raw_shape = None
                else:
                    if info is None or value.descriptor_ref != info.descriptor_ref:
                        return
                    getattr(codec, 'validate_' + kind)(info, value)
                    previous = self._latest[kind]
                    identity = value.state_id if kind == 'state' else value.touch_state_id
                    previous_id = None if previous is None else (
                        previous.state_id if kind == 'state' else previous.touch_state_id)
                    if previous_id is not None and identity <= previous_id:
                        return
                    if kind == 'state':
                        shape = tuple(len(getattr(value, n)) for n in ('raw_pos', 'raw_vel', 'raw_current', 'raw_status'))
                        if self._raw_shape is not None and shape != self._raw_shape:
                            raise ValueError('raw columns resized within a DescriptorRef')
                        self._raw_shape = shape
                self._latest[kind] = value
                self._errors[kind] = None
                callback = self._callbacks[kind]
                self._condition.notify_all()
        except (ValueError, TypeError, AttributeError, IndexError, OverflowError, struct.error) as error:
            with self._condition:
                if not self._closed:
                    self._errors[kind] = error
                    self._condition.notify_all()
            return
        if kind == 'info' and value.touch_available:
            try:
                self._register('touch')
            except Exception as error:
                with self._condition:
                    if not self._closed:
                        self._errors['touch'] = error
                        self._condition.notify_all()
        with self._condition:
            if self._closed or self._fatal is not None:
                return
        # An already selected callback is in flight; user code runs outside SDK
        # locks and may close this client. It never owns or closes the connection.
        if callback is not None:
            callback(value)

    def _get(self, kind):
        with self._condition:
            self._ensure_open()
            if self._errors[kind] is not None:
                raise ValueError('invalid EE ' + kind + ' feedback') from self._errors[kind]
            return self._latest[kind]

    @property
    def info(self):
        return self._get('info')

    @property
    def state(self):
        return self._get('state')

    @property
    def touch(self):
        return self._get('touch')

    def _wait(self, kind, timeout, after):
        deadline = _deadline(timeout)
        with self._condition:
            while True:
                value = self._get(kind)
                if kind == 'touch' and self._latest['info'] is not None and not self._latest['info'].touch_available:
                    raise ValueError('endpoint has no touch capability')
                if value is not None:
                    identity = value.descriptor_ref if kind == 'info' else value.sample_ref
                    if kind != 'info' and after is not None:
                        identity = (identity.descriptor_ref, identity.stream, identity.side, identity.sample_id)
                        after_identity = (after.descriptor_ref, after.stream, after.side, after.sample_id)
                    else:
                        after_identity = after
                    if after is None or identity != after_identity:
                        return value
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('timed out waiting for EE ' + kind)
                self._condition.wait(remaining)

    def wait_info(self, timeout=5.0, *, after_descriptor=None):
        if after_descriptor is not None:
            if not isinstance(after_descriptor, DescriptorRef):
                raise TypeError('after_descriptor must be DescriptorRef')
            codec._uint(after_descriptor.robot_runtime_id, 64, 'robot_runtime_id')
            codec._uint(after_descriptor.descriptor_epoch, 64, 'descriptor_epoch')
        return self._wait('info', timeout, after_descriptor)

    def _after_sample(self, sample, stream):
        if sample is not None:
            if not isinstance(sample, SampleRef) or sample.stream != stream or sample.side != self._side:
                raise ValueError('after_sample must identify this endpoint and stream')
            if not isinstance(sample.descriptor_ref, DescriptorRef):
                raise ValueError('after_sample has no DescriptorRef')
            for value, bits, name in ((sample.descriptor_ref.robot_runtime_id, 64, 'robot_runtime_id'),
                    (sample.descriptor_ref.descriptor_epoch, 64, 'descriptor_epoch'),
                    (sample.sample_id, 64, 'sample_id'), (sample.source_time_us, 64, 'source_time_us'),
                    (sample.validity, 8, 'validity')):
                codec._uint(value, bits, name)

    def wait_state(self, timeout=5.0, *, after_sample=None):
        self._after_sample(after_sample, SampleStream.EE_STATE)
        return self._wait('state', timeout, after_sample)

    def wait_touch(self, timeout=5.0, *, after_sample=None):
        self._after_sample(after_sample, SampleStream.EE_TOUCH)
        return self._wait('touch', timeout, after_sample)

    def publish(self, target, *, input_observations=()):
        with self._publish_lock:
            with self._condition:
                self._ensure_open()
                if self._read_only:
                    raise InvalidCommandError('read-only EndEffectorClient cannot publish')
                if not isinstance(input_observations, tuple) or input_observations:
                    raise InvalidCommandError('only K=0 is supported until the real startup observation binding is supplied')
                info = self._get('info')
                if info is None:
                    raise InvalidCommandError('validated EE Info is required before publish')
                try:
                    prepared = codec.prepare_target(info, target)
                except (ValueError, TypeError, OverflowError) as error:
                    raise InvalidCommandError(str(error)) from error
            if self._publisher is None:
                publisher = self._node.create_publisher_typed(
                    self._bindings.metas['cmd'], self._topic + '/cmd',
                    qos=self._bindings.aorta.QoS.realtime_control())
                with self._condition:
                    discard = self._closed
                    if not discard:
                        self._publisher = publisher
                if discard:
                    publisher.close()
                    raise ControllerClosedError('end-effector client closed during publisher creation')
            with self._condition:
                self._ensure_open()
                if self._latest['info'].descriptor_ref != info.descriptor_ref:
                    raise InvalidCommandError('DescriptorRef changed before reservation')
            context = self.connection._reserve_command()
            with self._condition:
                self._ensure_open()
                if self._latest['info'].descriptor_ref != info.descriptor_ref:
                    raise InvalidCommandError('DescriptorRef changed during reservation')
            submission = EndEffectorSubmission(
                info.descriptor_ref, info.layout_id, info.side,
                ControlRef(context.client_id, context.lease_id),
                CommandKey(context.session_id, context.sequence), prepared)
            self._publisher.publish_typed(
                lambda builder, header: codec.fill_command(self._bindings, submission, builder, header))
            return submission

    def close(self):
        with self._publish_lock, self._condition:
            if self._closed:
                return
            self._closed = True
            handles = [*self._subscriptions.values()]
            if self._publisher is not None:
                handles.append(self._publisher)
            self._subscriptions.clear()
            self._publisher = None
            self._callbacks = {'info': None, 'state': None, 'touch': None}
            self._condition.notify_all()
        for handle in handles:
            try:
                handle.close()
            except Exception:
                pass

    def __enter__(self):
        with self._condition:
            self._ensure_open()
        return self

    def __exit__(self, *_exc):
        self.close()
