# Python SDK ↔ C++ Mock latency CI

This benchmark measures the Python SDK and C++ Mock over isolated TCP loopback
with exact-ID correlation and a sequential rate sweep. It requires the pinned
binary wheels and compiler inputs, without robot connectivity or hardware.

## What runs

`SDK latency report` runs on every PR update (and merge-group/manual events),
without path filters. Each of **50, 100, 200, …, 1000 Hz** runs for at least
**60 seconds**, sequentially, without perf/strace/py-spy. A C++ process publishes
44-slot `HumanoidLowState` (self-contained motor states, two IMUs, command echo);
the current checkout's Python SDK decodes it, constructs 42 `MotorCommand`s,
and publishes `LowCmd`. The Mock verifies the FlatBuffer and copies all 42 slots.
This measures the generated C++ decoder/copy endpoint, **not** the complete
locomotion admission/interpolation/actuator pipeline.

Both processes use real Aorta, sensor-data/realtime-control QoS defaults, unique
`/bench/sdk_latency/<session>/...` topics and peer sessions restricted to
`127.0.0.1`. Multicast/gossip scouting, SHM and compression are disabled. No
control session, actuator command on a production topic, service or timer is
started. Synthetic positions are correlated by session, sequence, cmd_id and
validated 42-slot content, never by nearest timestamp. Scheduling does not
intentionally burst to catch up. Capture is bounded to 65,000 rows per window;
process watchdogs only terminate the benchmark's own process groups.

## Dependencies and reproducibility

- Python wheels come from this repository's `dependency_release_tag` in
  `compatibility.json` (`v0.1.0-rc.4`), verified against exact SHA-256s.
  The five dependency wheels are Aorta SDK 2026.10.12 for both Linux
  architectures, aorta-msgs 2026.10.12, vbot-humanoid-msgs 0.1.0, and
  FlatBuffers 25.9.23. CI uses this repository's `contents: read` token.
  Tests import `python/locomotion_aorta` from the checkout under test.
- C++ links the installed wheel's `libaorta_core.so` (ABI 13). `aorta_abi.h`
  contains the required public declarations for Aorta 2026.10.12. Size/offset
  assertions and the runtime version check validate compatibility; the expected
  ABI comes from the manifest. Updating the Aorta pin requires reviewing this
  subset too.
- C++ message headers are generated from vbot-humanoid-msgs's actual BFBS,
  checked against the manifest's schema hashes. FlatBuffers 25.9.23 source is
  SHA256-pinned. CMake/g++ build flatc and the Mock. Reusing a build directory
  restores extracted sources from the verified archive while keeping the
  compiler cache.
- `build.json` records schema/runtime/binary hashes and compiler. `results.json`
  records checkout SHA, PR head, source hashes, runtime versions, environment,
  raw counts and every rate's statistics. PR runs normally test GitHub's merge
  checkout; it is identified separately from the PR head.

## Reports and interpretation

Open the **SDK latency report** job's Summary directly from the PR checks.
It displays proportional text bar charts for RTT, both directions' frequencies,
SDK publish-call time, and expandable eight-stage charts/tables. These use plain
GitHub Markdown, without an external image host, public data upload or PR write
permission. Bars share the stated scale and round to 1/8 character; exact numbers
remain visible. They are not interactive SVG charts.

The Summary also links to `sdk-latency-<run>-<attempt>` (30-day retention).
Download and unzip it for the full SVG/HTML presentation and raw evidence:

- `report.html`: self-contained Chinese eight-stage proportional time chart;
- `summary.md`, `results.json`: both directions' actual send/receive Hz, matched
  counts, lost state/reply counts, RTT P50/P95/P99/max and SDK publish-call time;
- per-rate `sent.csv`, `client.csv`, `received.csv`, `joined.csv`, argv, session
  configuration and process logs. No per-frame filesystem I/O in the timed path.

All measurement timestamps use the same host's `CLOCK_MONOTONIC`, in ns; report
latencies are ms. Rates use `(count - 1) / (last stamp - first stamp)` per stream;
counts and durations are retained so losses cannot be hidden by that estimator.
Stage bars use **means of the same matched rows**, whose sum is mean RTT.
Percentiles do not add. SDK publish-call time overlaps transport and is reported
separately. The two communication stages include Aorta/Zenoh scheduling, copies,
Python dispatch/GIL and kernel transport; they are not pure TCP latency.
RTT begins at the C++ Aorta byte-publication call; the synthetic state's
construction and serialization happen before that boundary and are excluded.

The initial gate requires successful processes, complete frequency windows,
valid exact-ID joins, nonempty data, monotonic stage order and a produced report.
It does **not** invent a latency/loss SLA: packet losses and throughput are
reported, not suppressed. Shared GitHub runner jitter is not a robot performance
guarantee. A short local smoke is marked `SMOKE_ONLY`, never full `PASS`.

## Local reproduction (Linux x86_64)

Install the pinned x86 release wheels in a separate Python 3.10+ environment,
then, from this repository:

```bash
PYTHONPATH=python:tests python -m unittest discover -s tests -v
python tests/latency/build_mock.py --build-dir build/latency
python tests/latency/run.py --build-dir build/latency \
  --output build/latency-report --seconds 60 \
  --rates 50 100 200 300 400 500 600 700 800 900 1000
```

Use a new report directory for each run; build dependencies may be reused at the
same build path. For a smoke only, use `--seconds 2 --rates 50 1000`. Preserve
failed reports rather than retrying until green. Run builds serially on shared build machines. No robot connectivity is needed.
