"""Offline model-span summaries. Never called by the control or writer threads."""
import argparse
import json
import math

from .reader import Reader
from .session import Kind, Outcome


def summarize_spans(path):
    reader = Reader(path)
    groups = {}
    for row in reader:
        if row['kind'] != Kind.USER_SPAN:
            continue
        names = reader.integrity.run_header['stages']
        # Stage zero denotes unknown; do not label it as a registered model.
        name = names[row['stage'] - 1] if row['stage'] else '<unknown>'
        key = (name, row['outcome'])
        if key not in groups:
            groups[key] = {'durations': [], 'producers': set(), 'observed': 0, 'invalid_duration': 0}
        group = groups[key]
        group['observed'] += 1
        group['producers'].add(row['producer'])
        if row['duration_us'] is not None:
            group['durations'].append(row['duration_us'])
        else:
            group['invalid_duration'] += 1
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
    return dict(capture_complete=reader.integrity.complete,
        issues=reader.integrity.issues, stages=result,
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
