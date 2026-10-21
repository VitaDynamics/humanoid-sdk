"""Shared demo views/diagnostics; no transport, policy or automatic publishing."""
NOT_MEASURED = 'NOT_MEASURED'


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
