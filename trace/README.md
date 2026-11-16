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
options to the connection constructor. Custom user spans still use the separate
public `Session` API; they are not required for ordinary SDK instrumentation.

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
The count/capacity and Python snapshot overhead still require measurement;
this is not strict real-time acceptance.

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
