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
and queue submission run in C++. Native Body/EE encoders reuse the values they
already read: `body_encode` / `endpoint_encode` capture the actual float32
encoding separately from the caller's original values at publish return.
Successful encoding is not successful submission, CAN transmission or motion.

Body/EE/Touch native decoders capture the owned native data once, before DTO
construction. An UNKNOWN decode event is not a verdict that the SDK accepted
it. Later accepted/rejected callbacks and cached reads carry an exact
`reference_id` to that recording's decode event, keeping sample identity,
publication identity and local receipt clocks separate. They do not walk the
joint columns or Touch payload again. A private, non-dataclass provenance slot
does not change constructors, equality, `asdict` or the wire contract. A copied
or foreign-recording DTO without matching provenance uses normal snapshot
capture; no nearest-time match or permanent second payload cache is used.
Snapshots are owned POD copies; the writer retains no Python references. The writer's
close/completion wait is not part of a normal model or publish call.

Unknown stage names, excessive nesting and exhausted producer channels mark
incomplete capture without stopping computation. A nested SDK call carries the
active span identity. Publishing after leaving the span does not automatically
attribute a command to that inference. Pass the exact inference ID when opening
the later publish span; use explicit result links for background work.

`Trace.start_span(name)` returns a one-use token with `span_id` and
`inference_id`. The receiving worker calls `token.finish(failed=False)`; this
does not propagate a TLS context or block the submitter. Calling twice or closing
with unfinished tokens marks capture incomplete. At most 1024 tokens can be
outstanding. `Trace.new_id()` allocates a result/input identity without counting
a model invocation. `Trace.mark(name, kind=Kind.TRACE_LINK, inference_id=...,
reference_id=...)` records the actual consumer edge. Numeric columns accept
bounded contiguous typed buffers; reuse them on hot paths. `Trace.define` records
bounded cold model/marker metadata, including model hash and I/O signatures
when the application has verified them. Do not invent missing metadata.

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
file incomplete. Channels bind to native thread-lifetime tokens, not Python
`threading.local()` alone: foreign ctypes callbacks can lose Python TLS on each
entry even on the same native thread. Such callbacks reuse their existing
channel without another registration lock acquisition. The bounded registry
never recycles a channel onto a later thread with a reused OS/Python ID.
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
TX or execution. `body_take` includes polling and waiting; `body_decode` measures
the selected conversion. `body_admission_gate` measures the original identity
gate call, while accepted/rejected feedback records retain its verdict.
`endpoint_callback` starts after Aorta's message parsing and includes SDK
validation/delivery; it is not network reception. Endpoint decode timing carries
the actual left/right stream. Cache reads keep original receipt/sample clocks.
Full model/input causal coverage, target performance and installed ARM acceptance
are tracked separately. Legacy policy JSONL retirement is still validation-gated.

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

This reports model spans in `stages` and SDK calls in `sdk_stages`, separately by
recorded stage, stream and outcome, with
producer IDs and nearest-rank p50/p95/p99 in microseconds. It reads the actual
FBS integrity ledger; incomplete input returns exit code 2 and keeps all issues.
Counts are observed spans, not a claim of total inference calls when records
were lost. Failed calls are separate from successful-call timing. The offline
summary does not run in the control, Planner or writer thread. `stages` may be
empty without model spans; ordinary Body/EE publish, encode/decode, read,
control entry/exit and Touch request timings are in `sdk_stages`. Entry events
and untimed feedback snapshots are not counted as completed calls. A missing
duration stays unknown; a recorded zero stays zero. Stream0 timings have no
endpoint attribution and are not assigned to the nearest State. Encoding is
inside publish, so those durations must not be added. These are SDK wall-time
measurements, not CAN/device response latency or a hard-real-time guarantee.
`producer_issues` lists the
actual loss/error counters rather than hiding an incomplete capture.

Native capture requires matching extensions (`CAPTURE_API_VERSION=3` and
`TRACE_CODEC_API_VERSION=1`); mismatches fail at Trace/connection construction.
The encoder, decoder, control admission and publication gates retain their
original algorithms and wire layouts. C++ reading Python DTOs keeps the GIL;
there is no blanket `gil_scoped_release` around object access. Tiny queue submits
do not repeatedly release/reacquire the GIL. The guard is not held across the original SDK call or user model. Python
property getters invoked during snapshot extraction can reenter; nested capture
is rejected nonblockingly and counted, while the original SDK operation retains
its behavior.

For SDK-owned native ONNX sessions, `native_inference` measures the actual
`Ort::Session::Run` call while the GIL is already released. It ends before GIL
reacquisition and Python result conversion, and carries the active span ID.
The offline summary reports `actor/native_run` alongside the outer `actor`
span (or the registered name). Their difference includes input preparation,
copying, scheduling and result conversion; it is **not** a measurement of GIL
wait alone. Calls without a matching parent are explicitly unattributed.
Failures keep the original ORT exception and a failed timing record. Other
Python model backends, including WTT's multi-input Planner, retain their outer
span only; Trace does not wrap or replace their inference implementation.

`benchmarks/trace_capture.py` compares the same synthetic capture inputs using
baseline and candidate packages. It runs separately from unit tests, writes
individual elapsed-us samples and the full loss ledger, and fails if a capture
is incomplete. It creates no Aorta node or control publisher. Use distinct
`--output` directories; do not infer target deadlines by subtracting independent
p99 values or by accepting dropped records.

## Offline HTML and MCAP

After installing the matching wheel with its `trace` extra:

```bash
python -m locomotion_aorta.trace.summary logs/run.ftrace
python -m locomotion_aorta.trace.report logs/run.ftrace --output reports/run
# Optional: existing matching MCAP files; never start a recorder or robot action.
python -m locomotion_aorta.trace.report logs/run.ftrace --output reports/run-mcap \
  --mcap recorder/part-0.mcap --mcap recorder/part-1.mcap
```

Open `reports/run/report/index.html` directly. CSS/JS are local files with no
CDN, server or JSON fetch requirement. The report includes per-stage/per-side/
per-outcome timing histograms and CDF, position observations, paginated command
and span details, exact command keys, source SHA256, CSV and integrity exports.
Large identities are strings in browser JSON. Chart buckets preserve extrema;
quantiles use every valid recorded duration, not the chart downsample.

The reader indexes records in SQLite with a 4 MiB page cache and disk sorting.
Input chunks and metadata are bounded. Detail pages contain at most 128 commands;
full events remain in the offline `events.jsonl` export. This export is not a
second runtime writer. Output directories must be new, and source inputs are
hashed before/after analysis. Incomplete files retain verified records and their
actual loss/corruption status, never a fabricated zero-duration sample.

MCAP import verifies embedded BFBS against the installed generated metadata and
root, then uses the existing SDK decoders. Supported roots are BodyCmd/BodyState,
EE Cmd/State/Info and ControlStatus. Unknown roots, mismatched schema, bad payloads,
missing footer and oversize chunks remain explicit. Only exact command-key or
valid EXTERNAL application-key sightings are linked. MCAP log/publish times are
converted to microseconds; they are not client receive stamps. BodyState lacks
control_id and EE State lacks an applied command key: neither observation alone
proves a command-linked new hardware sample. The report deliberately leaves
hardware E2E, physical completion and unprovided clock alignment unavailable.

For an already-existing same-host subprocess IPC, `Trace.observed_span` accepts
explicit measured `CLOCK_MONOTONIC` endpoints in microseconds. Register the
worker/process provenance first; do not supply remote/device clocks through
this API. `Trace.vector(name, existing_buffer, inference_id=...)` captures a
contiguous float32/64 observation or action in one native call, with at most
8192 elements split into owned 256-element chunks. Offset/total and shared
attempt identity permit exact reconstruction; missing chunks remain missing.
This avoids `.tolist()` and Python per-element conversion for model logging.

For Body feedback with a valid EXTERNAL/TRACKING applied key, the report can
measure the interval from C0 to an accepted same-clock SDK observation when
R/session/sequence resolves to exactly one recorded attempt. It labels this
`APPLICATION_KEY_INTERVAL_ONLY`: the State lacks control_id, so this diagnostic
is not promoted to a full authenticated association, hardware E2E or motion
completion. Policy/replay/safety-hold source types, ambiguous keys, unknown
clock mappings and invalid causal order do not receive an invented interval.
