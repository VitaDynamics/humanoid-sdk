# SDK-local FBS trace (development)

The connection owns recording. Enable it when constructing the existing SDK
controller; then use the normal SDK methods. End-effector clients inherit their
connection's recorder. No DemoTrace, publisher wrapper, monkeypatch, additional
subscriber or user-managed writer is required.

```python
from locomotion_aorta import PullController
from locomotion_aorta.end_effector import EndEffectorClient

with PullController(group="default", trace_output="new-capture.ftrace") as controller:
    reply = controller.get_robot_info()
    with EndEffectorClient("left", connection=controller) as gripper:
        # Existing Info/target validation and explicitly authorized entry remain.
        # controller.publish(body_command)
        # gripper.publish(target, input_observations=selected_refs)
        pass
print(controller.trace_health)
```

`ExternalController` and read-only `PullStateSource` accept the same
`trace_output` and `trace_compression="raw"|"zstd"` options. Omitting the path
starts no trace resources and does not import the trace backend. Existing
Body/end-effector CLI entries forward their `--trace=fbs --trace-output=...`
options to the connection constructor. Custom user spans use the same file
without another recorder. Register names once and measure the actual model call:

```python
with PullController(trace_output="run.ftrace", trace_stages=("actor",)) as controller:
    with controller.trace_span("actor", iteration_id=1) as span:
        action = model.infer(observation)
```

This records monotonic start, elapsed microseconds, a distinct inference ID,
iteration ID and success/failure. The model exception remains unchanged.
Observation preparation and publication are outside this particular span.
Endpoint quantization uses a conservative +1 us margin; this is not a measured
performance guarantee. An asynchronous GPU/NPU call measures submission unless
its original completion wait is inside the span. Trace adds no synchronization.
When disabled, `trace_span` is a no-op and reads no trace clock.

For a run with model workers that outlive the SDK connection, create the same
recorder at the outer run boundary and lend it to the connection:

```python
from locomotion_aorta.trace import Trace

with Trace("run.ftrace", stages=("planner_infer", "planner_map")) as trace:
    with PullController(trace_recorder=trace) as controller:
        # SDK calls record automatically, just as with trace_output.
        with trace.span("planner_infer"):
            prediction = planner.run(inputs)
    # Join model workers here, before leaving the Trace context.
```

Borrowing connections never close this recorder, including constructor failure.
Register stages on the recorder, not again on the connection. First registration
uses a nonblocking guard; each thread then has its own fixed SPSC queue. Model
execution does not hold a capture lock. The single native writer owns all file
writes; entering from Python and reading Python DTO fields still incurs GIL
and scheduling overhead. Clocks, IDs, bounded call/span stacks, field snapshots
and queue submission now run in C++. BodyState native arrays are read directly
through checked buffers without Python per-axis iteration or lazy joints.
Snapshots are owned POD copies; the writer retains no Python references. The writer's
close/completion wait is not part of a normal model or publish call.

Unknown stage names, excessive nesting and exhausted producer channels mark
incomplete capture without stopping computation. A nested SDK call carries the
active span identity. Publishing after leaving the span does not automatically
attribute a command to that inference; asynchronous consumption linkage is
separate work.

The SDK records publish entry before validation/control-lock waiting,
reservation completion with the actual allocated key, and publication return
or failure. Failed sends retain their reservation event; no retry is introduced.
Pull reads preserve the returned sample and original clocks, including cached
IDs. Existing accepted Body/EE/Touch callbacks are observed outside the original
control/cache locks. Only already-subscribed Touch is recorded; trace does not
turn on touch or request control. RPC return alone is not a new Touch sample.

One connection has one native writer and eight preallocated producer channels,
bound lazily to actual calling/callback threads. The nonblocking binding and
capture guards drop observation on contention/reentry/exhaustion and mark the
file incomplete. Channels are never recycled onto another Thread object.
The count/capacity and complete target control-loop overhead still require
measurement; host capture microbenchmarks are not strict real-time acceptance.

Discovery/Info is cold metadata, split into bounded UTF-8 packets at the
`discovery` stage. Integer columns are `[byte_offset, total_bytes, words...]`;
words encode little-endian bytes, trimmed to the declared total. The outer
producer/attempt pair groups a document. Cold unknown nonfinite limits use null
plus their original-value map; validity masks remain unchanged. Command/state
column layouts, original clock meanings and field-mask semantics are in the
run header. Attempt IDs are local to a producer, not global command identities.

All trace times are integer microseconds. Publication return is not physical
TX or execution. Receive take/decode sub-stages, full model causal association,
WTT migration, MCAP/HTML and installed current-source/ARM/performance acceptance
remain pending. Legacy policy JSONL retirement is still validation-gated.

Each preallocated SPSC ring drops new packets when full. One native writer
drains continuously, builds FlatBuffers, optionally compresses with Zstd, and
writes checksummed bounded blocks without acquiring the GIL. Closing has a
bounded wait; a timed-out worker retains its own storage until it exits.
Writer errors and capture failures do not initiate CANCEL or replace a model
exception. Check `controller.trace_health` (or `Session.health()`) and the reader's final integrity status: a file
with drops, corruption, missing footer or unpersisted events is incomplete.
`close()` completing is not a disk-fsync or physical execution guarantee.

Files are created exclusively. The container starts with `FTRACE13`; each
32-byte little-endian header stores encoded/raw lengths, CRC32 of the raw
FlatBuffer, raw/Zstd codec, chunk sequence and uint64 base time. Event timestamps
use uint32 microsecond offsets with explicit uint64 extensions. The FBS root
identifier is `VTR1`; the checked-in header is generated with FlatBuffers
25.12.19, matching the existing vendor headers:

```bash
flatc --cpp --scoped-enums -o trace/generated trace/schema/trace.fbs
```

Independent host validation requires CPython 3.10 development headers,
pybind11 2.13.6, zlib and Zstd development libraries. This target deliberately
does not configure the SDK's existing ORT/Aorta targets; it does not prove the
complete wheel is installable. The full SDK CMake build retains those gates.

```bash
cmake -S trace -B build-trace -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build-trace --parallel 2
```

Queue capacity 512 and the 1-second flush default remain candidates. Capture
overhead, real-time performance and device end-to-end latency are not measured.

Offline model timing (after closing the run):

```bash
python -m locomotion_aorta.trace.summary run.ftrace
```

This reports observed spans separately by registered stage and outcome, with
producer IDs and nearest-rank p50/p95/p99 in microseconds. It reads the actual
FBS integrity ledger; incomplete input returns exit code 2 and keeps all issues.
Counts are observed spans, not a claim of total inference calls when records
were lost. Failed calls are separate from successful-call timing. The offline
summary does not run in the control, Planner or writer thread.

Native capture requires its matching extension (`CAPTURE_API_VERSION=1`), checked
before file creation. No receive/encoding/publication algorithm or wire layout
was changed by this optimization. Low-level codec/transport sub-stage timestamps
remain a separate pending integration; current snapshots describe the original
SDK call/accepted-feedback boundaries. C++ reading Python DTOs keeps the GIL;
there is no blanket `gil_scoped_release` around object access. Tiny queue submits
do not repeatedly release/reacquire the GIL. The guard is not held across the original SDK call or user model. Python
property getters invoked during snapshot extraction can reenter; nested capture
is rejected nonblockingly and counted, while the original SDK operation retains
its behavior.

`benchmarks/trace_capture.py` compares the same synthetic capture inputs using
baseline and candidate packages. It runs separately from unit tests, writes
individual elapsed-us samples and the full loss ledger, and fails if a capture
is incomplete. It creates no Aorta node or control publisher. Use distinct
`--output` directories; do not infer target deadlines by subtracting independent
p99 values or by accepting dropped records.
