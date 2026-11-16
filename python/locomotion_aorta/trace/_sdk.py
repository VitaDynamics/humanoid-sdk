"""Private observation of SDK calls. No user-side publisher or controller wrapper."""
from array import array
from dataclasses import asdict
import json
import math
import threading
from contextlib import nullcontext

from .session import Kind, Outcome, Session, monotonic_us

_NO_SPAN = nullcontext()

STAGES = ('body_read', 'body_publish', 'endpoint_feedback', 'endpoint_publish',
          'control_enter', 'control_exit', 'touch_received', 'key_reserved',
          'discovery', 'touch_request', 'body_callback')
BODY_COMMAND_COLUMNS = ('q', 'dq', 'tau', 'kp', 'kd')
BODY_STATE_COLUMNS = ('q', 'dq', 'tau_est', 'error_code')
EE_COMMAND_COLUMNS = ('pos', 'vel', 'vel_limit', 'tau', 'current')
EE_STATE_COLUMNS = ('pos', 'vel', 'tau', 'current', 'temperature_c', 'error_code')
EE_COMMAND_BITS = (1, 2, 4, 8, 16)
EE_STATE_BITS = (1, 2, 8, 16, 32, 128)


def _info_metadata(info):
    """Cold-only JSON representation; masked unknown limits may contain NaN."""
    nonfinite = {}

    def convert(value, path=''):
        if isinstance(value, float) and not math.isfinite(value):
            nonfinite[path] = repr(value)
            return None
        if isinstance(value, dict):
            return {key: convert(item, path + '.' + key) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [convert(item, path + '.' + str(i)) for i, item in enumerate(value)]
        return value

    return convert(asdict(info)), nonfinite


class SdkTrace:
    """Connection-owned writer with a fixed number of preallocated thread channels.

    Thread registration never waits; exhaustion is reported as incomplete capture.
    A retained Thread object prevents recycled Python/OS IDs from reusing a ring.
    """
    def __init__(self, path, *, compression='zstd', threads=8, stages=()):
        self.user_stages = tuple(stages)
        self.body_indices = {}
        self._registration = threading.Lock()
        self._local = threading.local()
        self._threads = []
        self.session = Session(path, producers=tuple('thread_' + str(i) for i in range(threads)),
            stages=STAGES + self.user_stages, compression=compression,
            streams=({'id': 1, 'name': 'body'}, {'id': 2, 'name': 'left'}, {'id': 3, 'name': 'right'}),
            metadata={
                'capture_scope': 'SDK method entry, reservation, return and accepted feedback',
                'thread_capacity': threads,
                'attempt_identity': ['producer', 'attempt_id'],
                'discovery_layout': ['byte_offset', 'total_bytes', 'little_endian_words'],
                'touch_request_semantics': 'RPC entry/return; not a new feedback sample',
                'column_order': 'field-major, in the original fixed axis order',
                'validity_order': 'one byte per value, followed by one byte per point time',
                'layouts': {
                    'body_command': BODY_COMMAND_COLUMNS,
                    'body_state': BODY_STATE_COLUMNS,
                    'endpoint_command': EE_COMMAND_COLUMNS,
                    'endpoint_command_integers': ['axis_count', 'axis_modes_follow',
                        'axis_command_field_masks_follow'],
                    'endpoint_state': EE_STATE_COLUMNS,
                    'endpoint_integers': ['axis_count', 'side', 'layout_id',
                        'sample_status', 'status_valid_mask', 'ready',
                        'output_allowed', 'inhibit_reason', 'touch_enabled',
                        'snapshot_role', 'axis_field_masks_follow'],
                    'body_state_integers': ['axis_count', 'applied_key_valid',
                        'association_source_type'],
                    'body_command_integers': ['axis_count', 'protocol_version',
                        'required_fields', 'required_fields_known',
                        'target_slot_mask', 'target_scope_known'],
                    'control_integers': ['client_id', 'generation', 'status_seq',
                        'reject_count', 'ingress_drop_count', 'timeout_hold'],
                    'sample_link_integers': ['source_runtime_id', 'stream', 'side', 'validity'],
                    'touch_payload_integers': ['layout_id', 'side', 'byte_offset',
                        'total_bytes', 'little_endian_uint64_words_follow'],
                    'touch_region_integers': ['layout_id', 'side', 'region_index',
                        'sample_seq', 'status_flags', 'direction_flags', 'validity',
                        'publish_time_valid'],
                    'touch_region_times': ['request', 'hardware_acquired', 'source', 'ingested', 'publish'],
                    'body_command_times': ['client_created', 'client_send', 'input_state_source'],
                    'body_state_times': ['state_source', 'state_publish', 'consumed',
                        'control_capture', 'first_output', 'client_receipt_monotonic',
                        'client_receipt_realtime', 'pull_context_receive',
                        'joint_acquired_ingested_assembled_follow'],
                    'endpoint_state_times': ['state_source', 'state_publish',
                        'hardware_acquired', 'ingested', 'assembled', 'client_position_receipt'],
                },
                'point_time_domains': {
                    'client_created': 'mapped Vita when available',
                    'client_send': 'mapped Vita when available',
                    'client_receipt_monotonic': 'client CLOCK_MONOTONIC',
                    'client_position_receipt': 'client CLOCK_MONOTONIC',
                    'client_receipt_realtime': 'client CLOCK_REALTIME',
                    'pull_context_receive': 'Core context stamp; not proven network arrival',
                    'other_source_points': 'original robot/driver domain; no mapping assumed',
                },
                'stream_ids': {'body': 1, 'left': 2, 'right': 3},
                'limitations': ['application return is not physical output',
                    'receive take/decode timestamps remain pending',
                    'repeated cached sample IDs remain repeated',
                    'feedback records are observations, not validity/arrival verdicts',
                    'Touch captures existing accepted callbacks; no extra subscription',
                    'capture overhead and installed ARM wheel not measured'],
            })
        self._channels = tuple(_Capture(self, self.session.producer('thread_' + str(i)))
                               for i in range(threads))

    def _channel(self):
        if self.session._closed:
            return None
        current = getattr(self._local, 'channel', None)
        if current is not None:
            return current
        if not self._registration.acquire(False):
            self._channels[0]._invalid()
            return None
        try:
            if len(self._threads) == len(self._channels):
                self._channels[0]._invalid()
                return None
            channel = self._channels[len(self._threads)]
            self._threads.append(threading.current_thread())
            self._local.channel = channel
            return channel
        finally:
            self._registration.release()

    def call(self, operation, function, *args, **kwargs):
        channel = self._channel()
        if channel is None:
            return function(*args, **kwargs)
        return channel.call(operation, function, args, kwargs)

    def span(self, name, *, iteration_id=0, inference_id=0):
        channel = self._channel()
        if channel is None:
            return _NO_SPAN
        if (name not in self.user_stages or len(channel._spans) >= 8 or
                type(iteration_id) is not int or not 0 <= iteration_id < 2**64 or
                type(inference_id) is not int or not 0 <= inference_id < 2**64):
            channel._invalid()
            return _NO_SPAN
        sequence = channel.producer.next_id()
        if sequence >= 2**56:
            channel._invalid()
            return _NO_SPAN
        identity = ((channel.producer.index + 1) << 56) | sequence
        return _UserSpan(channel, name, iteration_id, inference_id or identity, identity)

    def reserved(self, context, side=0):
        channel = self._channel()
        if channel is not None:
            channel._record(Kind.STAGE_EVENT, 'key_reserved', channel._stamp(), Outcome.OK,
                stream=1 + int(side), attempt=channel._attempts[-1] if channel._attempts else 0,
                snapshot=channel._reservation, subject=context)

    def received(self, kind, value):
        channel = self._channel()
        if channel is None:
            return
        if kind == 'state':
            channel.endpoint_feedback(value, None, None, None)
        elif kind == 'touch':
            channel.touch_received(value)
        elif kind == 'body':
            channel._record(Kind.BODY_FEEDBACK, 'body_callback', channel._stamp(), Outcome.OK,
                            stream=1, snapshot=channel._body_state, subject=value)
        elif kind == 'info':
            channel.info(value, 'endpoint_info')

    def discovery(self, reply):
        if reply.info is not None and reply.info.body is not None:
            self.body_indices = {name: i for i, name in enumerate(reply.info.body.joint_names)}
        channel = self._channel()
        if channel is not None:
            channel.info(reply, 'robot_info_reply')

    def close(self):
        try:
            if not self.session._closed:
                for channel in self._channels:
                    if channel._attempts or channel._spans:
                        channel._invalid()
            self.session.close()
        except Exception:
            self._channels[0]._invalid()

    def health(self):
        return self.session.health()


class _Capture:
    def __init__(self, owner, producer):
        self.owner, self.producer = owner, producer
        self.stages = {name: owner.session.stage(name) for name in STAGES + owner.user_stages}
        self._attempts = []
        self._spans = []
        self._lock = threading.Lock()
        self._values = array('d', [0.]) * 256
        self._validity = array('B', [0]) * 256
        self._integers = array('Q', [0]) * 32
        self._times = array('Q', [0]) * 128
        self._nv = self._ni = self._nt = 0

    def _invalid(self):
        self.producer.note_invalid()

    def _stamp(self):
        try:
            return monotonic_us()
        except Exception:
            self._invalid()
            return None

    def _value(self, value, valid=True):
        if self._nv >= 256:
            raise ValueError('trace value layout exceeds bound')
        if valid and (isinstance(value, bool) or not isinstance(value, (int, float))
                      or not math.isfinite(value)):
            raise ValueError('invalid trace numeric value')
        self._values[self._nv] = value if valid else 0.
        self._validity[self._nv] = bool(valid)
        self._nv += 1

    def _integer(self, value):
        if self._ni >= 32:
            raise ValueError('trace integer layout exceeds bound')
        self._integers[self._ni] = value
        self._ni += 1

    def _time(self, value):
        if self._nt >= 128 or not 0 <= value < 2**64:
            raise ValueError('invalid trace point time')
        self._times[self._nt] = value
        self._nt += 1

    def _record(self, kind, stage, stamp, outcome, *, attempt=0, stream=0,
                duration=None, snapshot=None, subject=None):
        if stamp is None:
            return
        if not self._lock.acquire(False):
            self._invalid()
            return
        try:
            self._nv = self._ni = self._nt = 0
            ids = [0] * 12
            if self._spans:
                current = self._spans[-1]
                ids[:4] = [current.iteration_id, current.inference_id,
                           current.identity, current.parent]
            ids[4] = attempt
            if snapshot is not None:
                subject = snapshot(subject, ids)
            if self._nv + self._nt > 256:
                raise ValueError('trace validity layout exceeds bound')
            for index in range(self._nt):
                self._validity[self._nv + index] = bool(self._times[index])
            self.producer.emit(kind, timestamp_us=stamp, duration_us=duration,
                stage=self.stages[stage], outcome=outcome, stream=stream, ids=ids,
                values=memoryview(self._values)[:self._nv],
                integers=memoryview(self._integers)[:self._ni],
                times_us=memoryview(self._times)[:self._nt],
                validity=memoryview(self._validity)[:self._nv + self._nt])
        except Exception:
            self._invalid()
        finally:
            self._lock.release()

    def _call(self, kind, stage, call, *, stream=0, before=None, after=None, subject=None):
        attempt = self.producer.next_id()
        start = self._stamp()
        self._record(kind, stage, start, Outcome.UNKNOWN, attempt=attempt,
                     stream=stream, snapshot=before, subject=subject)
        self._attempts.append(attempt)
        try:
            result = call()
        except BaseException:
            end = self._stamp()
            self._record(kind, stage, end, Outcome.FAILED, attempt=attempt, stream=stream,
                         duration=end-start+1 if start is not None and end is not None and end >= start else None)
            raise
        finally:
            self._attempts.pop()
        end = self._stamp()
        if start is not None and end is not None and end < start:
            self._invalid()
        self._record(kind, stage, end, Outcome.OK, attempt=attempt, stream=stream,
                     duration=end-start+1 if start is not None and end is not None and end >= start else None,
                     snapshot=after, subject=result)
        return result

    def _body_command(self, command, ids, required_fields, target_names):
        ids[6:11] = [command.runtime_id or 0, command.control_id,
                    command.session_id, command.sequence, command.input_state_id]
        self._integer(len(command.joints))
        self._integer(command.protocol_version)
        self._integer(0 if required_fields is None else required_fields)
        self._integer(int(required_fields is not None))
        slots = 0
        if target_names is not None:
            for name in target_names:
                slots |= 1 << self.owner.body_indices[name]
        self._integer(slots)
        self._integer(int(target_names is not None))
        for field in BODY_COMMAND_COLUMNS:
            for joint in command.joints:
                self._value(getattr(joint, field))
        for value in (command.ts_client_created_us, command.ts_client_send_us,
                      command.input_state_ts_real_us):
            self._time(value)

    def _body_state(self, state, ids):
        ids[6] = state.runtime_id
        ids[8:11] = [state.applied_session_id, state.applied_sequence, state.state_id]
        n = len(state.joints) if not hasattr(state, 'q') else len(state.q)
        if n != 31:
            raise ValueError('Body trace requires the original 31 slots')
        self._integer(n)
        self._integer(int(state.applied_key_valid))
        self._integer(state.association_source_type)
        for field in BODY_STATE_COLUMNS:
            column = getattr(state, field, None)
            if column is None:
                for joint in state.joints:
                    self._value(float(getattr(joint, field)))
            else:
                if len(column) != n:
                    raise ValueError('invalid Body trace column')
                for value in column:
                    self._value(float(value))
        for value in (state.ts_state_real_us, state.ts_state_pub_us, state.ts_consumed_us,
                      state.ts_control_us, state.ts_first_output_us,
                      getattr(state, 'received_monotonic_ns', 0) // 1000,
                      getattr(state, 'received_realtime_ns', 0) // 1000,
                      getattr(state, 'pull_context_receive_stamp_ns', 0) // 1000):
            self._time(value)
        stamps = getattr(state, 'joint_stamps_us', None)
        if stamps is not None:
            for row in stamps:
                for value in row:
                    self._time(int(value))
        else:
            for joint in state.joints:
                for field in ('ts_acquired_hw_us', 'ts_ingested_us', 'ts_assembled_us'):
                    self._time(getattr(joint, field))

    def _endpoint_target(self, target, _ids):
        self._integer(len(target.mode))
        for field, bit in zip(EE_COMMAND_COLUMNS, EE_COMMAND_BITS):
            column = getattr(target, field)
            for i, mask in enumerate(target.command_fields):
                selected = bool(mask & bit)
                self._value(column[i] if selected and column is not None else None, selected)
        for value in target.mode:
            self._integer(value)
        for value in target.command_fields:
            self._integer(value)

    def _endpoint_command(self, command, ids):
        ids[6:10] = [command.runtime_id, command.control_id,
                     command.key.session_id, command.key.sequence]
        self._endpoint_target(command.target, ids)
        self._time(command.ts_client_created_us)
        self._time(command.ts_client_send_us)

    def _endpoint_state(self, data, ids):
        state, received, role = data
        ids[6], ids[10] = state.runtime_id, state.state_id
        for value in (len(state.valid_fields), state.side, state.layout_id,
                      state.sample_status, state.status_valid_mask, int(state.ready),
                      int(state.output_allowed), state.inhibit_reason, int(state.touch_enabled), role):
            self._integer(value)
        for value in state.valid_fields:
            self._integer(value)
        for field, bit in zip(EE_STATE_COLUMNS, EE_STATE_BITS):
            column = getattr(state, field)
            for i, mask in enumerate(state.valid_fields):
                self._value(column[i] if mask & bit else None, bool(mask & bit))
        for value in (state.ts_state_real_us, state.ts_state_pub_us, state.ts_acquired_hw_us,
                      state.ts_ingested_us, state.ts_assembled_us,
                      int(received * 1_000_000) if received is not None else 0):
            self._time(value)

    def _status(self, status, ids):
        ids[6], ids[7] = status.runtime_id, status.control_id
        for name in ('client_id', 'generation', 'status_seq', 'reject_count',
                     'ingress_drop_count', 'timeout_hold'):
            self._integer(int(getattr(status, name)))

    def call(self, operation, function, args, kwargs):
        if len(self._attempts) >= 8:
            self._invalid()
            return function(*args, **kwargs)
        before = after = subject = None
        stream = 0
        if operation == 'body_publish':
            kind, stream, subject = Kind.BODY_COMMAND, 1, args[0]
            def snapshot(value, ids):
                self._body_command(value, ids, kwargs.get('required_fields'), kwargs.get('target_names'))
            # Entry must not evaluate user DTO getters before SDK validation.
            # Capture the validated returned command; rejection keeps entry/failure.
            after = snapshot
        elif operation == 'endpoint_publish':
            kind, subject = Kind.EFFECTOR_COMMAND, args[0]
            stream = 2 if function.__self__.side == 'left' else 3
            after = self._endpoint_command
        elif operation == 'body_read':
            kind, stream, after = Kind.BODY_READ, 1, self._body_state
        elif operation == 'touch_request':
            kind = Kind.INPUT_EVENT
            stream = 2 if function.__self__.side == 'left' else 3
        else:
            kind, after = Kind.CONTROL_STATUS, self._status
        result = self._call(kind, operation, lambda: function(*args, **kwargs),
                            stream=stream, before=before, after=after, subject=subject)
        if operation == 'endpoint_publish':
            for ref in result.input_observations:
                self._record(Kind.TRACE_LINK, operation, self._stamp(), Outcome.OK,
                             stream=stream, snapshot=self._sample_link, subject=(result, ref))
        return result

    def _reservation(self, context, ids):
        ids[6:10] = [context.runtime_id, context.control_id, context.session_id, context.sequence]

    def info(self, value, label):
        # Discovery/Info is cold and bounded; never serialize a per-cycle DTO.
        try:
            data, nonfinite = _info_metadata(value)
            raw = json.dumps({'type': label, 'value': data, 'nonfinite': nonfinite},
                             allow_nan=False, ensure_ascii=True).encode()
            if len(raw) > 256 * 1024:
                raise ValueError('Info trace exceeds cold metadata bound')
            identity = self.producer.next_id()
            for offset in range(0, len(raw), 216):
                self._record(Kind.STREAM_INFO, 'discovery', self._stamp(), Outcome.OK,
                             attempt=identity, snapshot=self._info_part, subject=(raw, offset))
        except Exception:
            self._invalid()

    def _info_part(self, data, ids):
        raw, offset = data
        self._integer(offset)
        self._integer(len(raw))
        part = raw[offset:offset+216]
        for i in range(0, len(part), 8):
            self._integer(int.from_bytes(part[i:i+8], 'little'))

    def _sample_link(self, data, ids):
        command, ref = data
        ids[6:11] = [command.runtime_id, command.control_id,
                    command.key.session_id, command.key.sequence, ref.sample_id]
        for value in (ref.runtime_id, ref.stream, ref.side, ref.validity):
            self._integer(value)
        self._time(ref.source_time_us)

    def endpoint_feedback(self, latest, position, fault, received):
        # Retain which existing snapshot is used, including duplicate/cached IDs.
        for role, state in enumerate((latest, position, fault)):
            if state is not None:
                self._record(Kind.EFFECTOR_FEEDBACK, 'endpoint_feedback', self._stamp(),
                    Outcome.UNKNOWN, stream=1 + int(state.side),
                    snapshot=self._endpoint_state,
                    subject=(state, received if role == 1 else None, role))

    def touch_received(self, sample):
        if sample is not None:
            for index, region in enumerate(sample.region_stamps):
                self._record(Kind.STREAM_INFO, 'touch_received', self._stamp(), Outcome.UNKNOWN,
                    stream=1 + int(sample.side), snapshot=self._touch_region,
                    subject=(sample, index, region))
            for offset in range(0, len(sample.payload), 216):
                self._record(Kind.INPUT_EVENT, 'touch_received', self._stamp(), Outcome.UNKNOWN,
                    stream=1 + int(sample.side), snapshot=self._touch_payload,
                    subject=(sample, offset))

    def _touch_region(self, data, ids):
        sample, index, region = data
        ids[6], ids[10] = sample.runtime_id, sample.touch_state_id
        for value in (sample.layout_id, sample.side, index, region.sample_seq,
                      region.status_flags, region.direction_flags, region.validity,
                      int(sample.publish_time_valid)):
            self._integer(value)
        for value in (region.ts_request_us, region.ts_acquired_hw_us,
                      region.ts_source_us, region.ts_ingested_us, sample.ts_state_pub_us):
            self._time(value)

    def _touch_payload(self, data, ids):
        sample, offset = data
        ids[6], ids[10] = sample.runtime_id, sample.touch_state_id
        for value in (sample.layout_id, sample.side, offset, len(sample.payload)):
            self._integer(value)
        part = sample.payload[offset:offset+216]
        for start in range(0, len(part), 8):
            self._integer(int.from_bytes(part[start:start+8], 'little'))


class _UserSpan:
    def __init__(self, capture, name, iteration_id, inference_id, identity):
        self.capture, self.name = capture, name
        self.iteration_id, self.inference_id, self.identity = iteration_id, inference_id, identity
        self.parent = capture._spans[-1].identity if capture._spans else 0
        self.start = None

    def __enter__(self):
        self.start = self.capture._stamp()
        self.capture._spans.append(self)
        return self

    def __exit__(self, error_type, _error, _traceback):
        end = self.capture._stamp()
        try:
            if self.start is not None and end is not None and end >= self.start:
                self.capture._record(Kind.USER_SPAN, self.name, self.start,
                    Outcome.FAILED if error_type else Outcome.OK, duration=end-self.start+1)
            else:
                self.capture._invalid()
        finally:
            self.capture._spans.pop()
        return False

