"""Private observation of SDK calls. No user-side publisher or controller wrapper."""
from dataclasses import asdict
import json
import math
import threading
import uuid
from .. import _trace_context
from contextlib import nullcontext

from .session import Kind, Session, _native

_NO_SPAN = nullcontext()

STAGES = ('body_read', 'body_publish', 'endpoint_feedback', 'endpoint_publish',
          'control_enter', 'control_exit', 'touch_received', 'key_reserved',
          'discovery', 'touch_request', 'body_callback')
NATIVE_STAGES = ('body_encode', 'endpoint_encode', 'body_decode', 'endpoint_decode', 'touch_decode', 'native_inference')
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


class Trace:
    """Run-owned writer with a fixed number of preallocated thread channels.

    Thread registration never waits; exhaustion is reported as incomplete capture.
    A retained Thread object prevents recycled Python/OS IDs from reusing a ring.
    Pass this recorder as ``trace_recorder`` to a connection to capture its SDK
    calls automatically. Close connections and join model workers before exiting
    this context. A borrowing connection never closes the recorder.
    """
    def __init__(self, path, *, compression='zstd', threads=8, stages=()):
        if getattr(_native(), 'CAPTURE_API_VERSION', None) != 2:
            raise RuntimeError('Trace requires the matching native capture extension')
        self.user_stages = tuple(stages)
        self.body_indices = {}
        self._registration = threading.Lock()
        self._local = threading.local()
        self._threads = []
        self.session = Session(path, producers=tuple('thread_' + str(i) for i in range(threads)),
            stages=STAGES + self.user_stages + NATIVE_STAGES, compression=compression,
            streams=({'id': 1, 'name': 'body'}, {'id': 2, 'name': 'left'}, {'id': 3, 'name': 'right'}),
            metadata={
                'capture_scope': 'SDK method entry, reservation, return and accepted feedback',
                'thread_capacity': threads,
                'capture_implementation': 'native clocks/IDs/stacks/POD snapshots; Python DTO access holds GIL',
                'attempt_identity': ['producer', 'attempt_id'],
                'native_reuse': {
                    'decoded_payload': 'native decode event; not an admission verdict',
                    'reference_id': 'exact decoded event/group ID in this recorder',
                    'group_id': 'decoder caller attempt when available',
                    'linked_body_times': ['client_receipt_monotonic', 'client_receipt_realtime', 'pull_context_receive'],
                    'linked_endpoint_integers': ['snapshot_role'],
                    'linked_endpoint_times': ['position_receipt_monotonic'],
                    'linked_touch_integers': ['layout_id', 'side', 'publish_time_valid'],
                    'linked_touch_times': ['state_publish'],
                    'encode_values': 'actual float32 values; separate from requested values at publish return',
                    'native_inference': 'ORT Run only, before GIL reacquisition and Python result conversion',
                },
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
        try:
            self._channels = tuple(_Capture(self, self.session.producer('thread_' + str(i)))
                                   for i in range(threads))
        except BaseException:
            self.session.close()
            raise

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
        if (name not in self.user_stages or
                type(iteration_id) is not int or not 0 <= iteration_id < 2**64 or
                type(inference_id) is not int or not 0 <= inference_id < 2**64):
            channel._invalid()
            return _NO_SPAN
        return _UserSpan(channel, name, iteration_id, inference_id)

    def decode(self, function, *args, _stage='body_decode'):
        channel = self._channel()
        if channel is None:
            return function(*args)
        try:
            token = channel.native.begin(Kind.STAGE_EVENT, channel.stages[_stage], 0)
        except Exception:
            channel._invalid()
            token = 0
        previous = _trace_context.set_sink(channel.sink)
        try:
            result = function(*args)
        except BaseException:
            channel._finish(token, True, None, None, None)
            raise
        else:
            channel._finish(token, False, None, None, None)
            return result
        finally:
            _trace_context.set_sink(previous)

    def reserved(self, context, side=0):
        channel = self._channel()
        if channel is not None:
            channel.native.reserved(context, int(side), channel.stages['key_reserved'])

    def received(self, kind, value):
        channel = self._channel()
        if channel is None:
            return
        if kind == 'state':
            channel.endpoint_feedback(value, None, None, None)
        elif kind == 'touch':
            channel.touch_received(value)
        elif kind == 'body':
            channel.native.body_feedback(value, channel.stages['body_callback'], True)
        elif kind == 'info':
            channel.info(value, 'endpoint_info')

    def rejected(self, kind, value):
        channel = self._channel()
        if channel is None:
            return
        if kind == 'body':
            channel.native.body_feedback(value, channel.stages['body_callback'], False)
        elif kind == 'state':
            channel.native.endpoint_feedback(value, 0, None, channel.stages['endpoint_feedback'], False)
        elif kind == 'touch':
            channel.native.touch(value, channel.stages['touch_received'], False)

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
                    if channel.native.active:
                        channel._invalid()
            self.session.close()
        except Exception:
            self._channels[0]._invalid()

    def health(self):
        return self.session.health()

    def __enter__(self):
        if self.session._closed:
            raise RuntimeError('trace recorder is closed')
        return self

    def __exit__(self, *_):
        self.close()


def _validate_options(output, recorder, stages):
    # Called before transport construction: invalid options create no resources.
    if recorder is not None:
        if output is not None or stages:
            raise ValueError('trace_recorder owns its output and stage registration')
        if not isinstance(recorder, Trace) or recorder.session._closed:
            raise ValueError('trace_recorder must be an open Trace')


class _Capture:
    """Thin dispatch only. Native owns clocks, IDs, bounded stacks and snapshots."""
    def __init__(self, owner, producer):
        self.owner, self.producer = owner, producer
        self.stages = {name: owner.session.stage(name) for name in STAGES + owner.user_stages + NATIVE_STAGES}
        identity = uuid.UUID(owner.session.header['run_id']).int
        self.native = _native().SdkCapture(owner.session._writer, producer.index,
            tuple(self.stages[name] for name in NATIVE_STAGES), (identity >> 64, identity & ((1 << 64)-1)))
        self.sink = self.native.sink()

    def _invalid(self):
        self.producer.note_invalid()

    def call(self, operation, function, args, kwargs):
        if operation == 'body_publish':
            kind, stream = Kind.BODY_COMMAND, 1
        elif operation == 'endpoint_publish':
            kind, stream = Kind.EFFECTOR_COMMAND, 2 if function.__self__.side == 'left' else 3
        elif operation == 'body_read':
            kind, stream = Kind.BODY_READ, 1
        elif operation == 'touch_request':
            kind, stream = Kind.INPUT_EVENT, 2 if function.__self__.side == 'left' else 3
        else:
            kind, stream = Kind.CONTROL_STATUS, 0
        try:
            token = self.native.begin(kind, self.stages[operation], stream)
        except Exception:
            self._invalid()
            token = 0
        previous = _trace_context.set_sink(self.sink)
        try:
            result = function(*args, **kwargs)
        except BaseException:
            self._finish(token, True, None, None, None)
            raise
        finally:
            _trace_context.set_sink(previous)
        self._finish(token, False, result, kwargs.get('required_fields'), kwargs.get('target_names'))
        return result

    def _finish(self, token, failed, result, required, targets):
        try:
            self.native.end(token, failed, result, required, targets, self.owner.body_indices)
        except Exception:
            self._invalid()

    def info(self, value, label):
        # Cold metadata only; per-cycle state and command objects stay native.
        try:
            data, nonfinite = _info_metadata(value)
            raw = json.dumps({'type': label, 'value': data, 'nonfinite': nonfinite},
                             allow_nan=False, ensure_ascii=True).encode()
            self.native.info(raw, self.stages['discovery'])
        except Exception:
            self._invalid()

    def endpoint_feedback(self, latest, position, fault, received):
        for role, state in enumerate((latest, position, fault)):
            if state is not None:
                self.native.endpoint_feedback(state, role, received if role == 1 else None,
                                              self.stages['endpoint_feedback'], True)

    def touch_received(self, sample):
        if sample is not None:
            self.native.touch(sample, self.stages['touch_received'], True)


class _UserSpan:
    def __init__(self, capture, name, iteration_id, inference_id):
        self.capture, self.stage = capture, capture.stages[name]
        self.iteration_id, self.inference_id = iteration_id, inference_id
        self.identity = 0
        self.previous = None

    def __enter__(self):
        self.identity = self.capture.native.begin_span(self.stage, self.iteration_id, self.inference_id)
        self.inference_id = self.inference_id or self.identity
        self.previous = _trace_context.set_sink(self.capture.sink)
        return self

    def __exit__(self, error_type, _error, _traceback):
        try:
            self.capture.native.end_span(self.identity, error_type is not None)
        finally:
            _trace_context.set_sink(self.previous)
        return False
