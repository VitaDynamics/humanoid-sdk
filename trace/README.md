# SDK-local FBS trace (development)

This draft adds a native recording container and the public
`locomotion_aorta.trace` API. It does not change robot wire messages or grant
control ownership. Production SDK controller/receive hooks, full model causal
association, MCAP analysis, HTML reports, installed-wheel/ARM validation and
performance acceptance are still pending. The shared end-effector demo now
offers `--trace=fbs --trace-output=NEW.ftrace`; see its README for exact scope.
The policy demos' legacy logging has not yet been retired.

Register producer and stage handles before control. One producer belongs to one
actual OS thread. All timestamps and durations submitted to trace are integer
microseconds; unknown duration is `None`, and a valid zero remains zero.
Do not subtract timestamps from different clocks without a verified mapping.

```python
from locomotion_aorta.trace import Session

with Session('new-capture.ftrace', producers=('control',), stages=('model',)) as run:
    producer = run.producer('control')
    with producer.span(run.stage('model'), iteration_id=1, inference_id=1):
        result = model(observation)
```

Each preallocated SPSC ring drops new packets when full. One native writer
drains continuously, builds FlatBuffers, optionally compresses with Zstd, and
writes checksummed bounded blocks without acquiring the GIL. Closing has a
bounded wait; a timed-out worker retains its own storage until it exits.
Writer errors and capture failures do not initiate CANCEL or replace a model
exception. Check `run.health()` and the reader's final integrity status: a file
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
