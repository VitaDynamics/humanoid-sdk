"""New EE development client on one existing EXTERNAL connection.

Observation sources are fixed at startup; callers supply the actual sample refs
used by each invocation. No independent controller or sample synchronizer.
"""
import math
import struct
import threading
import time
from dataclasses import fields, is_dataclass, replace

from . import _end_effector_codec as codec
from .controller import ControllerClosedError, ExternalController, InvalidCommandError, NotRunningError, _deadline
from .pull import PullStateSource
from .end_effector_types import (
    CommandKey, ControlRef, DescriptorRef, EndEffectorSide, EndEffectorSubmission,
    SampleRef, SampleStream, Field, SampleStatus,
    EndEffectorSetTouchRequest, ServiceStatus, TouchSetReason,
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


def _publication_time(kind, value):
    valid = value.time_valid_mask & 2 if kind == 'state' else value.publish_time_valid
    return value.ts_state_pub_us if valid and value.ts_state_pub_us else None


def _same_sample(kind, old, new):
    """A publication may revoke validity, never supply a new measurement."""
    if kind == 'touch':
        if old.payload != new.payload:
            raise ValueError('same Touch sample changed payload')
        for a, b in zip(old.region_stamps, new.region_stamps):
            # Payload availability/staleness vary; native region identity does not.
            if (replace(a, validity=a.validity & ~65) != replace(b, validity=b.validity & ~65)
                    or b.validity & 1 and not a.validity & 1):
                raise ValueError('same Touch sample changed source or restored validity')
        return
    source = ('ts_state_real_us', 'ts_acquired_hw_us', 'ts_ingested_us', 'is_mock')
    if (any(getattr(old, name) != getattr(new, name) for name in source)
            or old.time_valid_mask & 13 != new.time_valid_mask & 13
            or old.sample_status != SampleStatus.OK and new.sample_status == SampleStatus.OK):
        raise ValueError('same State sample changed source or restored validity')
    for masks, columns in (
            ('valid_fields', codec._STATE_COLUMNS),
            ('raw_valid_fields', (('raw_pos', 1), ('raw_vel', 2),
                                  ('raw_current', 4), ('raw_status', 8)))):
        for index, (a, b) in enumerate(zip(getattr(old, masks), getattr(new, masks))):
            if b & ~a:
                raise ValueError('same State sample restored measurement validity')
            for name, bit in columns:
                if b & bit and _token(getattr(old, name)[index]) != _token(getattr(new, name)[index]):
                    raise ValueError('same State sample changed measurement')


def _new_publication(kind, old, new):
    old_id, new_id = old.sample_ref.sample_id, new.sample_ref.sample_id
    if new_id < old_id:
        return False
    before, after = _publication_time(kind, old), _publication_time(kind, new)
    if new_id > old_id:
        if before is not None and after is not None and after < before:
            raise ValueError('new sample publication clock regressed')
        return True
    if before is None or after is None:
        raise ValueError('same sample publication ordering is unavailable')
    if after < before:
        return False
    if after == before:
        if _token(old) != _token(new):
            raise ValueError('conflicting same sample publication')
        return False
    _same_sample(kind, old, new)
    return True


class TouchSetError(RuntimeError):
    """An actual service reply did not confirm the request; raw fields retained."""

    def __init__(self, response):
        self.response = response
        super().__init__(f'set_touch not confirmed: status={response.status} reason={response.reason} '
                         f'enabled={response.enabled} enabled_valid={response.enabled_valid}')


class EndEffectorClient:
    def __init__(self, side, *, connection, read_only=False,
                 on_info=None, on_state=None, on_touch=None, observation_sources=()):
        codec.validate_observation_sources(observation_sources)
        self._observation_sources = observation_sources
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
        self._service_clients = set()
        self._raw_shape = None
        self._position = self._fault = self._position_received_at = None
        try:
            self._register('info')
            self._register('state')
            with self._condition:
                self._ensure_open()
        except BaseException:
            self.close()
            raise

    @property
    def observation_sources(self):
        return self._observation_sources

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
                            self._position = self._fault = self._position_received_at = None
                else:
                    if info is None or value.descriptor_ref != info.descriptor_ref:
                        return
                    getattr(codec, 'validate_' + kind)(info, value)
                    previous = self._latest[kind]
                    if previous is not None and not _new_publication(kind, previous, value):
                        return
                    if kind == 'state':
                        shape = tuple(len(getattr(value, n)) for n in ('raw_pos', 'raw_vel', 'raw_current', 'raw_status'))
                        if self._raw_shape is not None and shape != self._raw_shape:
                            raise ValueError('raw columns resized within a DescriptorRef')
                        self._raw_shape = shape
                        if self._position is not None and value.state_id == self._position.state_id:
                            self._position = value  # Includes same-sample validity revocation.
                        elif value.state_id and all(mask & Field.POS for mask in value.valid_fields):
                            self._position = value
                            self._position_received_at = time.monotonic()
                        if any(mask & Field.ERROR_CODE for mask in value.valid_fields):
                            # A partial zero diagnosis cannot erase a fault on
                            # another axis. Keep one original DTO, never merge.
                            if (self._fault is None or
                                    any(mask & Field.ERROR_CODE and code for mask, code in
                                        zip(value.valid_fields, value.error_code)) or
                                    all(not (mask & Field.ERROR_CODE and code) or
                                        value.valid_fields[i] & Field.ERROR_CODE
                                        for i, (mask, code) in enumerate(zip(
                                            self._fault.valid_fields, self._fault.error_code)))):
                                self._fault = value
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

    def motion_feedback(self):
        """Atomic original DTOs: latest status, position, fault, position receipt.

        No fields or source timestamps are merged. Receipt is local monotonic
        time of the new position sample, not of later diagnostic publications.
        """
        with self._condition:
            return self._get('state'), self._position, self._fault, self._position_received_at

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
                        changed = identity.descriptor_ref != after.descriptor_ref
                        if changed and (identity.descriptor_ref.robot_runtime_id != after.descriptor_ref.robot_runtime_id
                                or identity.descriptor_ref.descriptor_epoch <= after.descriptor_ref.descriptor_epoch):
                            raise ValueError('after_sample is not from this or a prior descriptor epoch')
                        if identity.sample_id > 0 and (changed or identity.sample_id > after.sample_id):
                            return value
                    else:
                        if after is None or identity != after:
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
                info = self._get('info')
                if info is None:
                    raise InvalidCommandError('validated EE Info is required before publish')
                try:
                    codec.validate_observations(info.descriptor_ref, input_observations,
                                                sources=self._observation_sources)
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
                CommandKey(context.session_id, context.sequence), prepared, input_observations)
            self._publisher.publish_typed(
                lambda builder, header: codec.fill_command(self._bindings, submission, builder, header))
            return submission

    def _check_touch_context(self, info, context):
        # Pull reception and the shared controller lock must stay outside this
        # client's condition. This is a fresh local check, not backend admission.
        with self._condition:
            self._ensure_open()
        current = self.connection._control_context()
        with self._condition:
            self._ensure_open()
            latest = self._get('info')
            if latest is None or (latest.descriptor_ref, latest.layout_id, latest.side) != (
                    info.descriptor_ref, info.layout_id, info.side):
                raise InvalidCommandError('DescriptorRef changed during set_touch')
            if current != context:
                raise NotRunningError('control context changed during set_touch')

    def set_touch(self, enable: bool, timeout: float = 5.0):
        """Return the actual confirmed reply, never a new State/Touch sample.

        Matching and RPC each have timeout_ms; this is not a combined strict
        deadline. Core/matching exceptions propagate, while service failures
        raise TouchSetError carrying the unmodified decoded response.
        """
        if type(enable) is not bool:
            raise ValueError('enable must be boolean')
        _deadline(timeout)
        if timeout * 1000 >= 1 << 64:
            raise ValueError('timeout_ms must fit uint64')
        timeout_ms = math.ceil(timeout * 1000)
        codec._uint(timeout_ms, 64, 'timeout_ms')
        with self._condition:
            self._ensure_open()
            if self._read_only:
                raise InvalidCommandError('read-only EndEffectorClient cannot set_touch')
            info = self._get('info')
            if info is None or not info.touch_available:
                raise InvalidCommandError('validated Info with touch capability is required')
        context = self.connection._control_context()
        service = codec.load_touch_bindings()
        rpc = self._node.create_client_typed(
            self._topic + '/set_touch', service.response.EndEffectorSetTouchResponse.GetRootAs,
            request_schema_meta=service.request_meta,
            options=self._bindings.aorta.ClientOptions(timeout_ms=timeout_ms, allow_retry=False))
        registered = False
        try:
            with self._condition:
                self._ensure_open()
                self._service_clients.add(rpc)
                registered = True
            self._check_touch_context(info, context)
            matched = rpc.raw().wait_for_matching(timeout_ms)
            self._check_touch_context(info, context)
            if not matched:
                raise TimeoutError('EE set_touch service provider not found')
            request = EndEffectorSetTouchRequest(
                info.descriptor_ref, info.layout_id, info.side,
                ControlRef(context.client_id, context.lease_id), enable)
            def fill(builder, header):
                self._check_touch_context(info, context)
                return codec.fill_set_touch(self._bindings, service, request, builder, header)
            reply = codec.decode_set_touch(rpc.call(fill).decode())
            self._check_touch_context(info, context)
            codec.validate_set_touch_response(request, reply)
            if (reply.status != ServiceStatus.SUCCESS or reply.reason != TouchSetReason.NONE
                    or not reply.enabled_valid or reply.enabled != enable):
                raise TouchSetError(reply)
            return reply
        finally:
            with self._condition:
                owned = rpc in self._service_clients
                self._service_clients.discard(rpc)
            if owned or not registered:
                try:
                    rpc.close()
                except Exception:
                    pass

    def close(self):
        with self._publish_lock, self._condition:
            if self._closed:
                return
            self._closed = True
            handles = [*self._subscriptions.values(), *self._service_clients]
            if self._publisher is not None:
                handles.append(self._publisher)
            self._subscriptions.clear()
            self._service_clients.clear()
            self._publisher = None
            self._callbacks = {'info': None, 'state': None, 'touch': None}
            self._position = self._fault = self._position_received_at = None
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
