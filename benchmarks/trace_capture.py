#!/usr/bin/env python3
"""Capture-only host comparison; no Aorta nodes, publishers, models or devices.

Use the same script/input with baseline and candidate installed/staged packages.
This is a benchmark, not a unit-test or a real-time acceptance result.
"""
import argparse
from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np
from locomotion_aorta import BodyCmd, BodyJointCmd, BodyState
from locomotion_aorta.body_types import BodyJointState
from locomotion_aorta.state_arrays import BodyStateArrays
from locomotion_aorta.trace import Trace, _sdk, _trace_native
from locomotion_aorta.trace.reader import Reader


def measure(function, cycles):
    samples = []
    for index in range(cycles):
        # Give the single writer drain time outside the measured call. Queue
        # loss remains a failure, never an explanation for an apparently fast run.
        if index % 16 == 0:
            time.sleep(.01)
        began = time.perf_counter_ns()
        function()
        elapsed = time.perf_counter_ns() - began
        samples.append((elapsed + 999) // 1000)
    ordered = sorted(samples)
    def percentile(p):
        return ordered[math.ceil(len(ordered)*p/100)-1]
    return dict(samples_us=samples, p50_us=percentile(50), p95_us=percentile(95),
                p99_us=percentile(99), max_us=max(samples))


def inputs():
    body = BodyState(17, tuple(BodyJointState(0, 0, 0, .1, .2, .3, 0) for _ in range(31)), runtime_id=11)
    arrays = dict(q=np.full(31, .1, np.float32), dq=np.full(31, .2, np.float32),
        tau_est=np.full(31, .3, np.float32), error_code=np.zeros(31, np.uint32),
        joint_stamps_us=np.zeros((31, 3), np.uint64), imu_values=np.zeros((0, 13), np.float32),
        imu_stamps_us=np.zeros((0, 4), np.uint64), imu_location=np.zeros(0, np.uint8),
        imu_status=np.zeros(0, np.uint8), imu_state_present=np.zeros(0, np.uint8))
    for f in fields(BodyStateArrays):
        if f.name not in arrays:
            arrays[f.name] = getattr(body, f.name, 0)
    command = BodyCmd(tuple(BodyJointCmd(.1, 0., 0., 20., 1.) for _ in range(31)),
                      runtime_id=11, control_id=13, session_id=2, sequence=3)
    return command, body, BodyStateArrays(**arrays)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--cycles', type=int, default=1024)
    args = parser.parse_args()
    if not 32 <= args.cycles <= 100000:
        parser.error('--cycles must be in [32, 100000]')
    args.output.mkdir(parents=True, exist_ok=False)
    command, state, arrays = inputs()
    results = {}
    for case in ('model_span', 'body_command', 'body_state', 'body_arrays'):
        path = args.output/(case+'.ftrace')
        with Trace(path, stages=('actor',)) as trace:
            if case == 'model_span':
                def call():
                    with trace.span('actor'):
                        pass
            elif case == 'body_command':
                def call():
                    trace.call('body_publish', lambda command: command, command)
            else:
                selected = state if case == 'body_state' else arrays
                def call():
                    trace.call('body_read', lambda: selected)
            for _ in range(32):
                call()
            results[case] = measure(call, args.cycles)
        reader = Reader(path)
        for _ in reader:
            pass
        results[case].update(capture_complete=reader.integrity.complete,
                             issues=reader.integrity.issues,
                             ledger=reader.integrity.run_end)
    report = dict(label=args.label, cycles=args.cycles, host=platform.platform(),
        python=platform.python_version(),
        source_sha256=hashlib.sha256(Path(_sdk.__file__).read_bytes()).hexdigest(),
        native_sha256=hashlib.sha256(Path(_trace_native.__file__).read_bytes()).hexdigest(),
        clock_quantization='elapsed endpoints rounded up to integer us',
        scope='Synthetic capture-only calls on a shared host; no hardware, control-loop or ARM acceptance. '
              'Reported percentiles are individual call durations, not differences of percentiles.',
        results=results)
    (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({name: {k:v for k,v in data.items() if k not in ('samples_us', 'ledger')}
                      for name,data in results.items()}, indent=2))
    return 0 if all(r['capture_complete'] for r in results.values()) else 2


if __name__ == '__main__':
    raise SystemExit(main())
