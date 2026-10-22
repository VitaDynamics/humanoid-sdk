"""TONIGHT/071 JSONL contract. All S/L points use CLOCK_REALTIME nanoseconds."""

import json
import time


NOT_MEASURED = "NOT_MEASURED"


def realtime_ns() -> int:
    return time.time_ns()


class CycleLog:
    def __init__(self, path, metadata: dict) -> None:
        self.stream = open(path, "x", encoding="utf-8")
        self.overheads: list[int] = []
        self.write({"record_type": "run", "schema": "vita.velocity_mode_latency.v1",
                    "clock": "CLOCK_REALTIME", "started_ns": realtime_ns(), **metadata})

    def write(self, record: dict) -> int:
        started = realtime_ns()
        self.stream.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
        self.stream.flush()
        return realtime_ns() - started

    def cycle(self, record: dict) -> None:
        # A second record reports the cost of serializing/writing/flushing the
        # cycle record; it cannot be inserted into the already-written record.
        overhead = self.write({"record_type": "cycle", **record})
        self.overheads.append(overhead)
        extra = self.write({"record_type": "logging_overhead", "cycle": record["cycle"],
                           "cycle_record_write_ns": overhead})
        self.overheads[-1] += extra

    def close(self) -> None:
        import numpy as np

        self.write({"record_type": "logging_summary", "finished_ns": realtime_ns(),
                    "clock": "CLOCK_REALTIME",
                    "scope": "cycle and overhead JSON serialization/write/flush; excludes this summary",
                    "N": len(self.overheads),
                    "ms": dict(zip(("p50", "p95", "p99", "max"),
                                   [float(v) / 1e6 for v in
                                    (*np.percentile(self.overheads, [50, 95, 99]), max(self.overheads))]))
                    if self.overheads else NOT_MEASURED})
        self.stream.close()


"""Shared demo views/diagnostics; no transport, policy or automatic publishing."""
NOT_MEASURED = 'NOT_MEASURED'
# Existing live-demo lowstate window; policies must not refresh a frozen input.
LOWSTATE_WINDOW_S = .2


def feedback_age_ns(state, monotonic_ns, realtime_ns):
    """Return local cache age; producer and client clocks need not agree."""
    received = state.received_monotonic_ns
    if not received:
        raise ValueError('SDK must provide feedback receipt timestamps')
    age = monotonic_ns - received
    if age < 0:
        raise TimeoutError('feedback receipt timestamp is in the future')
    return age


def joint_columns(state):
    """Read native array columns without materializing 31 compatibility DTOs."""
    if hasattr(state, 'q') and hasattr(state, 'error_code'):
        return state.q, state.dq, state.error_code
    joints = state.joints
    return (tuple(j.q for j in joints), tuple(j.dq for j in joints),
            tuple(j.error_code for j in joints))


def state_log_fields(state):
    q, dq, errors = joint_columns(state)
    return dict(
        S0=(getattr(state, 'ts_state_pub_us', 0) * 1000 or NOT_MEASURED),
        S1=(getattr(state, 'received_realtime_ns', 0) or NOT_MEASURED),
        applied_session_id=getattr(state, 'applied_session_id', 0),
        applied_sequence=getattr(state, 'applied_sequence', 0),
        ts_first_output_us=getattr(state, 'ts_first_output_us', 0),
        joint_error_codes=[int(v) for v in errors],
        pull_dequeued_stamp_ns=(getattr(state, 'pull_context_receive_stamp_ns', 0)
                                or NOT_MEASURED))


def publish_log_fields(row, sent, timing=None):
    row['session_id'] = getattr(sent, 'session_id', NOT_MEASURED)
    row['sequence'] = getattr(sent, 'sequence', NOT_MEASURED)
    if timing is not None:
        row['publish_timing'] = timing
    # A same-session sequence delta, not measured motor-output latency.
    session, seq = row['session_id'], row['sequence']
    applied = row.get('applied_sequence', 0)
    row['applied_sequence_lag'] = (seq - applied if isinstance(seq, int)
        and isinstance(session, int) and session != 0
        and session == row.get('applied_session_id') and 0 <= applied <= seq
        else NOT_MEASURED)


def joy_log_fields(sample):
    if sample is None:
        return dict(joy_messages=0, last_joy_ns=None, last_joy_axes=())
    return dict(joy_messages=sample.samples_total,
                last_joy_ns=sample.received_realtime_ns, last_joy_axes=sample.axes,
                last_joy_buttons=sample.buttons,
                joy_pressed_counts=dict(sample.pressed_counts),
                joy_released_counts=dict(sample.released_counts),
                joy_missing_since_read=sample.missing_samples_since_read,
                joy_missing_total=sample.missing_samples_total,
                joy_sequence_available=sample.sequence_available,
                joy_event_history_complete=sample.event_history_complete,
                joy_sequence_diagnostics=dict(sample.sequence_diagnostics))
