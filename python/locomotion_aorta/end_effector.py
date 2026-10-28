"""EE development client on one confirmed EXTERNAL/HUMANOID_TELEOP connection.

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
    CommandKey, DescriptorRef, EndEffectorSide, EndEffectorSubmission,
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


def _publication_time(kind, value):
    valid = bool(value.ts_state_pub_us) if kind == 'state' else value.publish_time_valid
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
    source = ('ts_state_real_us', 'ts_acquired_hw_us', 'ts_ingested_us')
    if (any(getattr(new, name) not in (0, getattr(old, name)) for name in source)
            or old.sample_status != SampleStatus.OK and new.sample_status == SampleStatus.OK):
        raise ValueError('same State sample changed source or restored validity')
    for index, (a, b) in enumerate(zip(old.valid_fields, new.valid_fields)):
        if b & ~a:
            raise ValueError('same State sample restored measurement validity')
        for name, bit in codec._STATE_COLUMNS:
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
        super().__init__(f'set_touch not confirmed: status={response.status} reason={response.reason}')


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
        self._position = self._fault = self._position_received_at = None
        try:
            self.connection._register_end_effector_client(self)
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

    def _retire_descriptor_locked(self, error):
        self._fatal = error
        self._latest['state'] = self._latest['touch'] = None
        self._errors['state'] = self._errors['touch'] = None
        self._position = self._fault = self._position_received_at = None
        handles = list(self._service_clients)
        self._service_clients.clear()
        if self._publisher is not None:
            handles.append(self._publisher)
            self._publisher = None
        self._condition.notify_all()
        return handles

    @staticmethod
    def _close_handles(handles):
        for handle in handles:
            try:
                handle.close()
            except Exception:
                pass

    def _fail_descriptor(self, error):
        with self._condition:
            if self._closed or self._fatal is not None:
                return
            handles = self._retire_descriptor_locked(error)
        self._close_handles(handles)

    def _close_locked(self):
        if self._closed:
            return []
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
        return handles

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
            if kind == 'info':
                codec.validate_info(value)
                if value.side != self._side:
                    raise ValueError('Info side differs from endpoint')
                try:
                    if not self.connection._bind_end_effector_runtime(
                        self, value.descriptor_ref.robot_runtime_id
                    ):
                        return
                except (ValueError, ControllerClosedError) as error:
                    self._fail_descriptor(error)
                    return
            retired = None
            with self._publish_lock, self._condition:
                if self._closed or self._fatal is not None:
                    return
                info = self._latest['info']
                if kind == 'info' and info is not None and value.descriptor_ref != info.descriptor_ref:
                    retired = self._retire_descriptor_locked(ValueError(
                        'robot runtime changed; recreate the client on a new connection'))
                elif kind == 'info' and info is not None and _description(value) != _description(info):
                    retired = self._retire_descriptor_locked(ValueError(
                        'immutable Info changed within one robot runtime'))
                elif kind != 'info':
                    if info is None or value.descriptor_ref != info.descriptor_ref:
                        return
                    getattr(codec, 'validate_' + kind)(info, value)
                    previous = self._latest[kind]
                    if previous is not None and not _new_publication(kind, previous, value):
                        return
                    if kind == 'state':
                        if self._position is not None and value.state_id == self._position.state_id:
                            self._position = value  # Includes same-sample validity revocation.
                        elif value.state_id and all(mask & Field.POS for mask in value.valid_fields):
                            self._position = value
                            self._position_received_at = time.monotonic()
                        if any(mask & Field.ERROR_CODE for mask in value.valid_fields):
                            # A partial diagnosis cannot clear a known fault:
                            # another partial fault may have arrived meanwhile.
                            # Retain one original DTO until all axes are diagnosed.
                            if (self._fault is None or
                                    not any(mask & Field.ERROR_CODE and code for mask, code in
                                            zip(self._fault.valid_fields, self._fault.error_code)) or
                                    all(mask & Field.ERROR_CODE for mask in value.valid_fields)):
                                self._fault = value
                if retired is None:
                    self._latest[kind] = value
                    self._errors[kind] = None
                    callback = self._callbacks[kind]
                    self._condition.notify_all()
            if retired is not None:
                self._close_handles(retired)
                return
        except (ValueError, TypeError, AttributeError, IndexError, OverflowError, struct.error) as error:
            with self._condition:
                if not self._closed:
                    self._errors[kind] = error
                    self._condition.notify_all()
            return
        self._deliver(kind, value, callback)

    def _deliver(self, kind, value, callback):
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
                        if identity.descriptor_ref != after.descriptor_ref:
                            raise ValueError('after_sample is not from this static descriptor')
                        if identity.sample_id > after.sample_id:
                            return value
                    else:
                        if after is None or identity != after:
                            return value
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('timed out waiting for EE ' + kind)
                self._condition.wait(remaining)

    def wait_info(self, timeout=5.0, *, after_descriptor=None):
        """Wait for the first immutable Info publication for this robot process."""
        if after_descriptor is not None:
            if not isinstance(after_descriptor, DescriptorRef):
                raise TypeError('after_descriptor must be DescriptorRef')
            codec._uint(after_descriptor.robot_runtime_id, 64, 'robot_runtime_id')
            if not after_descriptor.robot_runtime_id:
                raise ValueError('robot_runtime_id must be nonzero')
        return self._wait('info', timeout, after_descriptor)

    def _after_sample(self, sample, stream):
        if sample is not None:
            if not isinstance(sample, SampleRef) or sample.stream != stream or sample.side != self._side:
                raise ValueError('after_sample must identify this endpoint and stream')
            if not isinstance(sample.descriptor_ref, DescriptorRef):
                raise ValueError('after_sample has no DescriptorRef')
            for value, bits, name in ((sample.descriptor_ref.robot_runtime_id, 64, 'robot_runtime_id'),
                    (sample.sample_id, 64, 'sample_id'), (sample.source_time_us, 64, 'source_time_us'),
                    (sample.validity, 8, 'validity')):
                codec._uint(value, bits, name)
            if not sample.descriptor_ref.robot_runtime_id:
                raise ValueError('robot_runtime_id must be nonzero')

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

            def prepare_send():
                self._ensure_open()
                if self._latest['info'].descriptor_ref != info.descriptor_ref:
                    raise InvalidCommandError('DescriptorRef changed before reservation')
                if self._publisher is None:
                    self._publisher = self._node.create_publisher_typed(
                        self._bindings.metas['cmd'], self._topic + '/cmd',
                        qos=self._bindings.aorta.QoS.realtime_control())
                return self._publisher

            runtime_id = info.descriptor_ref.robot_runtime_id
            context, publisher = self.connection._reserve_end_effector_command(
                self, runtime_id, prepare_send
            )
            submission = EndEffectorSubmission(
                info.descriptor_ref, info.layout_id, info.side,
                context.control_id,
                CommandKey(context.session_id, context.sequence), prepared, input_observations)

            def submit(current):
                self._ensure_open()
                if (current.client_id != context.client_id or
                        current.control_id != context.control_id or
                        current.generation != context.session_id or
                        current.request_id != context.request_id):
                    raise NotRunningError('control context changed before EE submission')
                latest = self._latest['info']
                if latest is None or (latest.descriptor_ref, latest.layout_id, latest.side) != (
                        info.descriptor_ref, info.layout_id, info.side):
                    raise InvalidCommandError('DescriptorRef changed during reservation')
                if self._publisher is not publisher:
                    raise InvalidCommandError('EE publisher retired before submission')
                publisher.publish_typed(
                    lambda builder, header: codec.fill_command(
                        self._bindings, submission, builder, header
                    )
                )

            self.connection._end_effector_control_context(self, runtime_id, submit)
            return submission

    def _check_touch_context(self, info, context=None, operation=None):
        def check(current):
            self._ensure_open()
            latest = self._get('info')
            if latest is None or (latest.descriptor_ref, latest.layout_id, latest.side) != (
                    info.descriptor_ref, info.layout_id, info.side):
                raise InvalidCommandError('DescriptorRef changed during set_touch')
            if context is not None and current != context:
                raise NotRunningError('control context changed during set_touch')
            return operation() if operation is not None else current

        return self.connection._end_effector_control_context(
            self, info.descriptor_ref.robot_runtime_id, check
        )

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
        context = self._check_touch_context(info)
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
                context.control_id, enable)
            def fill(builder, header):
                return self._check_touch_context(
                    info,
                    context,
                    lambda: codec.fill_set_touch(
                        self._bindings, service, request, builder, header
                    ),
                )
            reply = codec.decode_set_touch(rpc.call(fill).decode())
            self._check_touch_context(info, context)
            codec.validate_set_touch_response(request, reply)
            if reply.status != ServiceStatus.SUCCESS or reply.reason != TouchSetReason.NONE:
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
        handles = self.connection._close_end_effector_client(self)
        self._close_handles(handles)

    def __enter__(self):
        with self._condition:
            self._ensure_open()
        return self

    def __exit__(self, *_exc):
        self.close()
