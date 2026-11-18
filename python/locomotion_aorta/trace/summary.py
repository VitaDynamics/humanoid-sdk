"""Offline SDK-call and model-span timings; never runs in the control thread."""
import argparse
import json

from .index import TraceIndex
from .session import Kind, Outcome

_SDK_STAGE_KINDS = {
    'body_read': Kind.BODY_READ, 'body_publish': Kind.BODY_COMMAND,
    'endpoint_publish': Kind.EFFECTOR_COMMAND,
    'control_enter': Kind.CONTROL_STATUS, 'control_exit': Kind.CONTROL_STATUS,
    'touch_request': Kind.INPUT_EVENT,
    **{name: Kind.STAGE_EVENT for name in (
        'body_encode', 'endpoint_encode', 'body_decode', 'endpoint_decode', 'touch_decode',
        'body_take', 'body_admission_gate', 'endpoint_callback')},
}


def _timings(db, where, parameters):
    count = db.execute('SELECT count(duration) FROM timings WHERE ' + where, parameters).fetchone()[0]

    def at(rank):
        if not count:
            return None
        value, = db.execute('SELECT duration FROM timings WHERE ' + where +
                           ' AND duration IS NOT NULL ORDER BY duration LIMIT 1 OFFSET ?',
                           (*parameters, rank)).fetchone()
        return int(value)

    producers = [r[0] for r in db.execute('SELECT DISTINCT producer FROM timings WHERE ' +
                                         where + ' ORDER BY producer', parameters)]
    return dict(duration_samples=count, producers=producers, min_us=at(0),
                p50_us=at((count * 50 + 99) // 100 - 1),
                p95_us=at((count * 95 + 99) // 100 - 1),
                p99_us=at((count * 99 + 99) // 100 - 1), max_us=at(count - 1))


def summarize_index(index):
    """Exact disk-backed quantiles and parent joins, even for hours-long files."""
    db = index.db
    db.execute('DROP TABLE IF EXISTS timings')
    db.execute('''CREATE TEMP TABLE timings (
        category TEXT, stage TEXT, stream INTEGER, outcome INTEGER,
        producer INTEGER, duration TEXT, event_id INTEGER)''')
    db.execute("INSERT INTO timings SELECT 'model',stage,0,outcome,producer,duration,id FROM events WHERE kind=?",
               (Kind.USER_SPAN,))
    # Join by exact producer/span, independent of file persistence ordering.
    # Duplicate parents are ambiguous, never select a convenient matching name.
    db.execute('''INSERT INTO timings SELECT 'model',
        COALESCE((SELECT CASE WHEN count(*)=1 THEN min(p.stage) END FROM events p
                  WHERE p.kind=? AND p.producer=e.producer AND p.span=e.span),
                 '<unattributed>') || '/native_run',0,e.outcome,e.producer,e.duration,e.id
        FROM events e WHERE e.kind=? AND e.stage='native_inference' ''',
        (Kind.USER_SPAN, Kind.STAGE_EVENT))
    for stage, kind in _SDK_STAGE_KINDS.items():
        db.execute("INSERT INTO timings SELECT 'sdk',stage,stream,outcome,producer,duration,id FROM events "
                   'WHERE stage=? AND kind=? AND outcome!=?', (stage, kind, Outcome.UNKNOWN))
    db.execute('CREATE INDEX timing_group ON timings(category,stage,stream,outcome,duration)')
    names = {item['id']: item.get('name') for item in index.header.get('streams', [])
             if isinstance(item, dict) and type(item.get('id')) is int}
    models, sdk = [], []
    for category, stage, stream, outcome, count, valid in db.execute('''
            SELECT category,stage,stream,outcome,count(*),count(duration) FROM timings
            GROUP BY category,stage,stream,outcome ORDER BY category,stage,stream,outcome'''):
        group = dict(stage=stage, outcome=Outcome(outcome).name,
                     **_timings(db, 'category=? AND stage=? AND stream=? AND outcome=?',
                                (category, stage, stream, outcome)))
        if category == 'sdk':
            group.update(stream=stream, stream_name=names.get(stream),
                         observed_calls=count, invalid_duration_calls=count-valid)
            sdk.append(group)
        else:
            group.update(observed_spans=count, invalid_duration_spans=count-valid)
            models.append(group)
    issues = []
    for producer, counts in enumerate((index.integrity['run_end'] or {}).get('producers', [])):
        if (not counts['counters_valid'] or any(counts[name] for name in (
                'dropped_events', 'unpersisted_events', 'wrong_thread_events', 'invalid_events'))):
            issues.append(dict(producer=producer, **counts))
    return dict(capture_complete=index.integrity['complete'], issues=index.integrity['issues'],
        producer_issues=issues, stages=models, sdk_stages=sdk,
        sdk_stage_coverage={name: ('OBSERVED' if any(g['stage'] == name for g in sdk)
            else 'NOT_OBSERVED' if name in index.header.get('stages', []) else 'NOT_IN_CAPTURE_SCHEMA')
            for name in _SDK_STAGE_KINDS},
        scope='Observed model spans and completed SDK calls, separated by recorded stage/stream/outcome. '
              'Missing durations are not zero. Stream 0 has no endpoint attribution. '
              'Encoding is nested within publish: do not add their durations. '
              'SDK call duration is not transport, CAN, robot-admission or actuator latency. '
              'No cross-clock subtraction or nearest-feedback matching.',
        percentile_method='exact nearest rank; disk-backed sort',
        duration_semantics='monotonic endpoint difference plus 1us quantization margin')


def summarize_spans(path):
    with TraceIndex(path) as index:
        return summarize_index(index)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path')
    args = parser.parse_args()
    result = summarize_spans(args.path)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['capture_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
