# Python SDK ↔ C++ Mock standalone latency benchmark

This benchmark measures the Python SDK and C++ Mock over isolated TCP loopback
with exact-ID correlation. It requires the pinned
binary wheels and compiler inputs, without robot connectivity or hardware.

## What runs

`SDK latency report` runs on every PR update (and merge-group/manual events),
without path filters. The business-rate integration check is **50 Hz for
60 seconds**, separately from unit-test discovery. Higher-rate pressure
measurements require an explicit standalone invocation outside `tests/`; they
are not part of the automatic PR matrix. No perf/strace/py-spy is used.
A C++ process publishes
31-slot `BodyState` (fixed motor structs, two IMUs; applied echo omitted);
the current checkout's Python SDK decodes it, constructs31 `BodyJointCmd`s,
and publishes `BodyCmd`. The Mock verifies the FlatBuffer and copies all31 slots.
This measures the generated C++ decoder/copy endpoint, **not** the complete
locomotion admission/interpolation/actuator pipeline.

Both processes use real Aorta, sensor-data/realtime-control QoS defaults, unique
`/bench/sdk_latency/<session>/...` topics and peer sessions restricted to
`127.0.0.1`. Multicast/gossip scouting, SHM and compression are disabled. A synthetic typed `get_robot_info` RPC is served under the same isolated
benchmark prefix and queried once by the client. An isolated synthetic
PREPARE/COMMIT/RUNNING and CANCEL lifecycle uses the existing Mock Protocol;
no robot actuator topic is used. This is host RPC evidence, never robot readback.
Synthetic positions are joined by the actual selected input state ID and the
actual returned command session/sequence, with validated31-slot content, never
by nearest timestamp or the benchmark window ID. Scheduling does not
intentionally burst to catch up. Capture is bounded to 65,000 rows per window;
process watchdogs only terminate the benchmark's own process groups.

## Dependencies and reproducibility

- Python wheels come from this repository's `dependency_release_tag` in
  `compatibility.json` (`v0.1.0-rc.4`), verified against exact SHA-256s.
  The five dependency wheels are Aorta SDK 2026.10.20 for both Linux
  architectures, aorta-msgs 2026.10.20, the exact vbot-humanoid-msgs version
  in the manifest, and
  FlatBuffers 25.12.19. CI uses this repository's `contents: read` token.
  Each job builds and installs the checkout's SDK wheel before measuring.
  The client rejects source shadowing and records both SDK and native codec paths.
- C++ links the installed wheel's `libaorta_core.so` (ABI 13). `aorta_abi.h`
  contains the required public declarations for Aorta 2026.10.20. Size/offset
  assertions and the runtime version check validate compatibility; the expected
  ABI comes from the manifest. Updating the Aorta pin requires reviewing this
  subset too.
- C++ message headers are generated from vbot-humanoid-msgs's actual BFBS,
  checked against the manifest's schema hashes. FlatBuffers 25.12.19 source is
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

New captures directly sample `CLOCK_REALTIME` in both processes for all eight
stages and RTT (ns, reported as ms). The original `CLOCK_MONOTONIC` fields remain
in raw CSV for regression and within-process frequency/publish-call elapsed time.
There is no offset conversion. Historical captures without REALTIME remain readable
and are explicitly labeled legacy; incomplete REALTIME records still fail integrity.
Negative REALTIME differences/order inversions are counted and displayed, never a new
pass/fail threshold. Signed raw differences remain in CSV and statistics; clock steps
do not represent physical negative latency. All eight same-host MONOTONIC
stage boundaries must still be nondecreasing, including publish→decode,
bytes→callback and callback→copied, whether or not REALTIME columns exist.
A MONOTONIC ordering violation fails integrity; REALTIME inversions only count.
The eight original stage boundaries, full windows and ID/content gates remain. Rates use `(count - 1) / (last stamp - first stamp)` per stream;
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
build and install this checkout's SDK wheel (including its native codec), then,
from this repository:

```bash
PYTHONPATH=.:tests python -m unittest discover -s tests -v
python benchmarks/latency/build_mock.py --build-dir build/latency
python benchmarks/latency/run.py --build-dir build/latency \
  --output build/latency-report --seconds 60 \
  --rates 50
```

Use a new report directory for each run; build dependencies may be reused at the
same build path. For a smoke only, use `--seconds 2 --rates 50`. Preserve
failed reports rather than retrying until green. Run builds serially on shared build machines. No robot connectivity is needed.

## Independent state source integration

`tests/integration/state_source_probe.py --output <new-directory>` verifies real Aorta state-only
a lazy state-only subscription, snapshot object identity across repeated/reset
state_id, direct MONOTONIC/REALTIME receipt and snapshots surviving close. The per-frame file
handshake is integration evidence, not a speed benchmark. It uses isolated loopback
and private topics, one synthetic state publisher, no control publisher or lease.
It does not replace the 50 Hz integration check or establish device/aarch64 acceptance.

The 50 Hz integration check retains its required CI job. An independent parallel
job builds and installs the SDK wheel, runs SDK/native unit tests and the read-only
source probe. A failure in either job cannot skip the other job.

The encoding boundaries retain the CSV names `fill_ns` / `filled_ns`. They now
wrap the actual native encoder, including Python input copying, released-GIL
FlatBuffers construction and the returned bytes. They no longer instrument the
retained Python golden encoder. Clocks, joins and integrity gates are unchanged.


## Optional stress measurement

An explicitly requested standalone measurement may pass other rates with
`--rates`. The previous 50–1000 Hz matrix can still be reproduced manually
through this benchmark; it is not imported or executed by unit tests and is
not required for the 50 Hz business-rate check. A complete non-baseline run
is labeled `BENCHMARK_ONLY`; a short run remains `SMOKE_ONLY`. Process or
data-integrity failures remain `FAIL` at every rate. Changing the test scope
does not repair or erase a control-entry identity failure in an older run.

## Independent cadence correctness

The SDK still accepts independent endpoint rates. The original finite FakeClock
checks for different left/right periods and non-dividing Body/EE periods live
in `check_scheduler_cadences.py`. They reuse the existing example test fixture
and the two original method bodies, without
nodes, sockets, elapsed-time load or a frequency sweep. They are not discovered
by normal unit tests and are not added to automatic CI. Explicitly select them
from a development checkout with matching generated bindings when validating
that independent-rate API:

```bash
PYTHONPATH=python:tests python benchmarks/latency/check_scheduler_cadences.py
```

The normal unit cases exercise SDK command publication at 50 Hz. Native cadence
and synthetic feedback-ID advancement in those cases are separate input domains,
not a request to run the SDK at a higher frequency.
