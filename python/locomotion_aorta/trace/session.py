"""Thin, bounded native capture API; never imports an example or aorta runtime."""
from enum import IntEnum
import importlib
import json
import math
from pathlib import Path
import time
import uuid


class Kind(IntEnum):
    RUN_HEADER = 1
    STREAM_INFO = 2
    BODY_READ = 3
    BODY_COMMAND = 4
    BODY_FEEDBACK = 5
    EFFECTOR_COMMAND = 6
    EFFECTOR_FEEDBACK = 7
    STAGE_EVENT = 8
    CYCLE_OUTCOME = 9
    CONTROL_STATUS = 10
    INPUT_EVENT = 11
    GRIP_DECISION = 12
    MARKER_DEFINITION = 13
    MODEL_DEFINITION = 14
    USER_SPAN = 15
    USER_MARK = 16
    TRACE_LINK = 17
    INFERENCE_LIFECYCLE = 18
    RECORDER_HEALTH = 19
    ERROR = 20
    RUN_END = 21


class Outcome(IntEnum):
    UNKNOWN = 0
    OK = 1
    REJECTED = 2
    FAILED = 3
    CANCELLED = 4
    SHADOW = 5
    SKIPPED_DUPLICATE_STATE = 6
    SKIPPED_STALE_STATE = 7
    SKIPPED_OVERRUN = 8
    NOT_AVAILABLE = 9
    AMBIGUOUS = 10
    INVALID = 11


IDS = ('iteration_id', 'inference_id', 'span_id', 'parent_span_id', 'attempt_id',
       'group_id', 'runtime_id', 'control_id', 'session_id', 'command_sequence',
       'state_id', 'reference_id')
EMPTY_IDS = (0,) * len(IDS)


def monotonic_us():
    return time.monotonic_ns() // 1000


def _u64(value, name):
    if type(value) is not int or not 0 <= value < 1 << 64:
        raise ValueError(name + ' must be a nonnegative uint64')
    return value


def _metadata(value):
    """Cold-path validation, never a recursive per-cycle conversion."""
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError('trace metadata keys must be strings')
            if key.endswith(('_ns', '_ms', '_seconds')) and not isinstance(item, str):
                raise ValueError('trace metadata time values must use *_us')
            _metadata(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _metadata(item)
    elif isinstance(value, float) and not math.isfinite(value):
        raise ValueError('non-finite trace metadata')


def _native():
    try:
        module = importlib.import_module('locomotion_aorta.trace._trace_native')
    except ImportError as error:
        raise RuntimeError('FBS trace requires the matching installed native extension') from error
    if (module.FORMAT_MAJOR, module.FORMAT_MINOR) != (1, 3):
        raise RuntimeError('FBS trace native format version mismatch')
    return module


class _NullSpan:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False


class _Disabled:
    enabled = False

    def emit(self, *_, **__):
        return False

    def mark(self, *_, **__):
        return False

    def span(self, *_, **__):
        return _NULL_SPAN

    def close(self, **_):
        return True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False


_NULL_SPAN = _NullSpan()
disabled = _Disabled()


class Producer:
    enabled = True

    def __init__(self, session, index):
        self._session, self.index = session, index
        self.invalid_events = 0
        self._next_id = 1

    def next_id(self):
        result = self._next_id
        if result >= (1 << 64) - 1:
            self.enabled = False
            return 0
        self._next_id += 1
        return result

    def emit(self, kind, *, timestamp_us=None, duration_us=None, stream=0,
             stage=0, clock=0, epoch=1, outcome=Outcome.UNKNOWN, ids=EMPTY_IDS,
             values=None, integers=None, times_us=None, validity=None):
        if not self.enabled or self._session._closed:
            return False
        # No arbitrary property dictionaries or recursive DTO serialization.
        try:
            valid = timestamp_us is not None
            stamp = _u64(timestamp_us, 'timestamp_us') if valid else monotonic_us()
            duration = _u64(duration_us, 'duration_us') if duration_us is not None else 0
            return self._session._writer.submit(
                self.index, int(kind), stamp, duration, duration_us is not None,
                stream, stage, clock, epoch, int(outcome), ids,
                values, integers, times_us, validity, True)
        except Exception:
            # Optional observation must not replace a publication/control error.
            self.invalid_events += 1
            self._session._writer.note_invalid(self.index)
            return False

    def mark(self, stage, *, ids=EMPTY_IDS, outcome=Outcome.OK):
        return self.emit(Kind.USER_MARK, stage=stage, ids=ids, outcome=outcome)

    def span(self, stage, *, iteration_id=0, inference_id=0, parent_span_id=0):
        return _Span(self, stage, iteration_id, inference_id, parent_span_id)


class _Span:
    def __init__(self, producer, stage, iteration_id, inference_id, parent):
        self.producer, self.stage = producer, stage
        self.ids = (iteration_id, inference_id, producer.next_id(), parent, *EMPTY_IDS[4:])
        self.start_us = None

    def __enter__(self):
        self.start_us = monotonic_us()
        return self

    def __exit__(self, error_type, _error, _traceback):
        end_us = monotonic_us()
        if self.start_us is not None:
            # Conservative <=1us quantization margin, not hidden ns storage.
            duration = end_us - self.start_us + 1
            self.producer.emit(Kind.USER_SPAN, timestamp_us=self.start_us,
                duration_us=duration if duration > 0 else None, stage=self.stage,
                ids=self.ids, outcome=Outcome.FAILED if error_type else Outcome.OK)
        return False


class Session:
    """One native writer, preallocated per-producer rings, one exclusive file.

    512 slots is a candidate capacity, not a measured acceptance result. Names
    and stream/stage/model definitions are frozen before the writer starts.
    """
    enabled = True

    def __init__(self, path, *, producers=('control',), stages=(), streams=(),
                 models=(), metadata=None, capacity=512, compression='zstd',
                 flush_interval_us=1_000_000, drain_interval_us=1_000,
                 writer_cpus=(), writer_nice=0, memory_budget_bytes=64 * 1024 * 1024):
        names = tuple(producers)
        stages = tuple(stages)
        if not names or len(names) > 32 or len(set(names)) != len(names):
            raise ValueError('trace requires 1..32 unique producer names')
        if len(stages) > 256 or len(set(stages)) != len(stages):
            raise ValueError('trace requires at most 256 unique stages')
        if any(not isinstance(s, str) or not s or len(s.encode()) > 128 for s in (*names, *stages)):
            raise ValueError('trace names must be bounded nonempty strings')
        if compression not in ('raw', 'zstd'):
            raise ValueError('trace compression must be raw or zstd')
        header = dict(metadata or {})
        header.update(record_type='run_header', format_major=1, format_minor=3,
                      run_id=str(uuid.uuid4()), process_instance=str(uuid.uuid4()),
                      clock_domains=[dict(id=0, name='CLOCK_MONOTONIC', device='client', epoch=1)],
                      producers=names, stages=stages, streams=streams, models=models,
                      started_us=monotonic_us(), flush_interval_us=flush_interval_us,
                      drain_interval_us=drain_interval_us,
                      span_duration_semantics='quantized timestamp difference plus 1us upper bound',
                      writer_requested_cpus=tuple(writer_cpus), writer_requested_nice=writer_nice,
                      capacity_candidate_not_validated=capacity)
        _metadata(header)
        self._closed = False
        self._writer = _native().Session(str(Path(path)), len(names), capacity,
            compression == 'zstd', json.dumps(header, ensure_ascii=True, allow_nan=False),
            flush_interval_us, drain_interval_us, list(writer_cpus), writer_nice,
            memory_budget_bytes)
        if not self._writer.wait_ready(200_000):
            self._writer.close(200_000)
            raise RuntimeError('trace writer initialization failed: ' + self._writer.status()['error'])
        self._producers = {name: Producer(self, index) for index, name in enumerate(names)}
        self._stages = {name: index + 1 for index, name in enumerate(stages)}
        self.header = header
        self.close_complete = None

    def producer(self, name):
        return self._producers[name]

    def stage(self, name):
        return self._stages[name]

    def close(self, *, timeout_us=200_000):
        if self._closed:
            return self.close_complete
        _u64(timeout_us, 'timeout_us')
        self._closed = True
        self.close_complete = self._writer.close(timeout_us)
        return self.close_complete

    def health(self):
        result = json.loads(self._writer.health_json())
        result.update(self._writer.status())
        result['invalid_events'] = {name: p.invalid_events for name, p in self._producers.items()}
        return result

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
        return False
