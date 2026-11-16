"""Trace the existing demo calls without owning control or reading more feedback.

These are application call boundaries. They do not claim transport receipt,
internal key-reservation time, physical TX, or hardware execution.
"""
from array import array
from dataclasses import asdict
import math
import threading

from .session import Kind, Outcome, Session, monotonic_us


STAGES = ('body_read', 'body_publish', 'endpoint_feedback', 'endpoint_publish',
          'control_enter', 'control_exit', 'touch_confirmation')
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


class DemoTrace:
    """One run/file/writer; all hooks here run on the existing control thread.

    Buffer preparation has a nonblocking observation-only lock so synchronous
    getter reentry cannot overwrite a packet. No lock is held across SDK calls.
    """
    def __init__(self, path, robot_reply, *, compression='zstd', capacity=512):
        info, nonfinite_info = _info_metadata(robot_reply.info)
        self._body_indices = {} if robot_reply.info.body is None else {
            name: index for index, name in enumerate(robot_reply.info.body.joint_names)}
        self.session = Session(path, producers=('control',), stages=STAGES,
            compression=compression, capacity=capacity,
            streams=({'id': 1, 'name': 'body'}, {'id': 2, 'name': 'left'},
                     {'id': 3, 'name': 'right'}),
            metadata={
                'capture_scope': 'demo application calls; no extra reads or subscribers',
                'robot_runtime_id': robot_reply.runtime_id,
                'robot_info': info,
                'info_nonfinite_fields': nonfinite_info,
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
                        'sample_seq', 'status_flags', 'direction_flags', 'validity'],
                    'touch_region_times': ['request', 'hardware_acquired', 'source', 'ingested'],
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
                    'no internal key-reservation or take/decode timestamps',
                    'repeated cached sample IDs remain repeated',
                    'feedback records are observations, not validity/arrival verdicts',
                    'touch payload is captured at confirmation, not a continuous subscription',
                    'capture overhead and installed ARM wheel not measured'],
            })
        self.producer = self.session.producer('control')
        self.stages = {name: self.session.stage(name) for name in STAGES}
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
        try:
            result = call()
        except BaseException:
            end = self._stamp()
            self._record(kind, stage, end, Outcome.FAILED, attempt=attempt, stream=stream,
                         duration=end-start+1 if start is not None and end is not None and end >= start else None)
            raise
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
                slots |= 1 << self._body_indices[name]
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

    def read_body(self, controller, timeout):
        return self._call(Kind.BODY_READ, 'body_read', lambda: controller.wait_state(timeout),
                          stream=1, after=self._body_state)

    def publish_body(self, controller, command, **kwargs):
        def snapshot(value, ids):
            return self._body_command(value, ids, kwargs.get('required_fields'), kwargs.get('target_names'))
        return self._call(Kind.BODY_COMMAND, 'body_publish',
            lambda: controller.publish(command, **kwargs), stream=1,
            before=snapshot, after=snapshot, subject=command)

    def publish_endpoint(self, client, target, observations):
        result = self._call(Kind.EFFECTOR_COMMAND, 'endpoint_publish',
            lambda: client.publish(target, input_observations=observations),
            stream=2 if client.side == 'left' else 3,
            before=self._endpoint_target, after=self._endpoint_command, subject=target)
        # Selected references come from the actual returned submission, never latest.
        for ref in result.input_observations:
            self._record(Kind.TRACE_LINK, 'endpoint_publish', self._stamp(), Outcome.OK,
                stream=2 if client.side == 'left' else 3,
                snapshot=self._sample_link, subject=(result, ref))
        return result

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

    def enter(self, controller, timeout):
        return self._call(Kind.CONTROL_STATUS, 'control_enter',
                          lambda: controller.enter_external(timeout), after=self._status)

    def exit(self, controller, timeout):
        return self._call(Kind.CONTROL_STATUS, 'control_exit',
                          lambda: controller.exit_external(timeout), after=self._status)

    def touch_confirmation(self, state, sample):
        self._record(Kind.EFFECTOR_FEEDBACK, 'touch_confirmation', self._stamp(), Outcome.OK,
            stream=1 + int(state.side), snapshot=self._endpoint_state, subject=(state, None, 0))
        if sample is not None:
            for index, region in enumerate(sample.region_stamps):
                self._record(Kind.STREAM_INFO, 'touch_confirmation', self._stamp(), Outcome.UNKNOWN,
                    stream=1 + int(sample.side), snapshot=self._touch_region,
                    subject=(sample, index, region))
            for offset in range(0, len(sample.payload), 216):
                self._record(Kind.INPUT_EVENT, 'touch_confirmation', self._stamp(), Outcome.UNKNOWN,
                    stream=1 + int(sample.side), snapshot=self._touch_payload,
                    subject=(sample, offset))

    def _touch_region(self, data, ids):
        sample, index, region = data
        ids[6], ids[10] = sample.runtime_id, sample.touch_state_id
        for value in (sample.layout_id, sample.side, index, region.sample_seq,
                      region.status_flags, region.direction_flags, region.validity):
            self._integer(value)
        for value in (region.ts_request_us, region.ts_acquired_hw_us,
                      region.ts_source_us, region.ts_ingested_us):
            self._time(value)

    def _touch_payload(self, data, ids):
        sample, offset = data
        ids[6], ids[10] = sample.runtime_id, sample.touch_state_id
        for value in (sample.layout_id, sample.side, offset, len(sample.payload)):
            self._integer(value)
        part = sample.payload[offset:offset+216]
        for start in range(0, len(part), 8):
            self._integer(int.from_bytes(part[start:start+8], 'little'))

    def __enter__(self):
        return self

    def __exit__(self, error_type, _error, _traceback):
        if error_type is not None:
            self._record(Kind.ERROR, 'control_exit', self._stamp(), Outcome.FAILED)
        try:
            self.session.close()
        except Exception:
            # Optional recording cleanup cannot replace an in-flight control fault.
            self._invalid()

    def health(self):
        return self.session.health()
