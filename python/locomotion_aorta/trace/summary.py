"""Offline model-span summaries. Never called by the control or writer threads."""
import argparse
import json
import math

from .reader import Reader
from .session import Kind, Outcome


def summarize_spans(path):
    reader = Reader(path)
    groups = {}
    parents = {}
    native = []

    def observe(name, outcome, duration, producer):
        key = (name, outcome)
        if key not in groups:
            groups[key] = {'durations': [], 'producers': set(), 'observed': 0, 'invalid_duration': 0}
        group = groups[key]
        group['observed'] += 1
        group['producers'].add(producer)
        if duration is not None:
            group['durations'].append(duration)
        else:
            group['invalid_duration'] += 1

    for row in reader:
        names = reader.integrity.run_header['stages']
        name = names[row['stage'] - 1] if row['stage'] else '<unknown>'
        if row['kind'] == Kind.USER_SPAN:
            parents[row['producer'], row['span_id']] = name
            observe(name, row['outcome'], row['duration_us'], row['producer'])
        elif row['kind'] == Kind.STAGE_EVENT and name == 'native_inference':
            native.append((row['producer'], row['span_id'], row['outcome'], row['duration_us']))
    # Exact producer/span linkage, never nearest-time matching. A parent may be
    # persisted later; missing parents remain explicitly unattributed.
    for producer, span, outcome, duration in native:
        name = parents.get((producer, span), '<unattributed>') + '/native_run'
        observe(name, outcome, duration, producer)
    result = []
    for (stage, outcome), group in sorted(groups.items()):
        values = sorted(group['durations'])
        def percentile(percent):
            return values[math.ceil(len(values) * percent / 100) - 1] if values else None
        result.append(dict(stage=stage, outcome=Outcome(outcome).name,
            observed_spans=group['observed'], duration_samples=len(values),
            invalid_duration_spans=group['invalid_duration'], producers=sorted(group['producers']),
            min_us=values[0] if values else None, p50_us=percentile(50),
            p95_us=percentile(95), p99_us=percentile(99), max_us=values[-1] if values else None))
    producer_issues = []
    for index, counts in enumerate((reader.integrity.run_end or {}).get('producers', [])):
        if (not counts['counters_valid'] or any(counts[name] for name in (
                'dropped_events', 'unpersisted_events', 'wrong_thread_events', 'invalid_events'))):
            producer_issues.append(dict(producer=index, **counts))
    return dict(capture_complete=reader.integrity.complete,
        issues=reader.integrity.issues, producer_issues=producer_issues, stages=result,
        scope='Observed completed spans only; missing spans are not zero-duration calls. '
              'Separate stages/outcomes; no cross-clock subtraction or physical-output claim.',
        percentile_method='nearest rank',
        duration_semantics='monotonic endpoint difference plus 1us quantization margin')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path')
    args = parser.parse_args()
    result = summarize_spans(args.path)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['capture_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
