# Optional demo ONNX backend

SDK 0.2.1 embeds the pybind11/Microsoft ORT 1.20.1 CPU backend directly in
its only distribution, `locomotion-aorta`. There is no standalone backend
package and no pure Python SDK wheel for this release. The inference backend is loaded only when a demo constructs a model;
SDK core imports do not load it. The `teleop-demo` extra contains NumPy and
pygame; there is no Python ORT fallback.

See the [SDK README](../README.md) for the supported hosts and release scope.

## Local build and release audit

Use CMake 3.20+ and a C++17 compiler. Obtain the architecture-matching official
CPU archive from `ort-archives.json`; CMake verifies its SHA before extraction.
There are no implicit downloads or dependencies on a robot's `/app` filesystem.

```sh
python tools/fetch_ort.py --output /absolute/path/to/official-archive.tgz
python -m pip install build==1.2.2.post1 scikit-build-core==0.11.6 pybind11==2.13.6 ninja==1.11.1.3
CMAKE_ARGS='-DORT_ARCHIVE=/absolute/path/to/official-archive.tgz' \
  python -m build --wheel --no-isolation --outdir dist/raw .
python -m pip install -r demo_onnx/requirements-release.txt
python demo_onnx/tools/wheels.py repair --wheel <raw-wheel> --arch x86_64 --output <new-audited-dir>
```

Raw `linux_<arch>` wheels are local build intermediates, not release assets.
Repair uses auditwheel with the owner-selected `manylinux_2_35_<arch>` policy;
it validates the binary and metadata instead of renaming a file. Actual symbol
requirements may be older than the product baseline; the tag never claims an
older environment than the audited binary supports. The pinned auditwheel version detects the wheel target architecture for cross
builds. The tool calls the pinned repair entrypoint with the requested target
because the upstream CLI's choices enumerate host platforms. Wheel/ELF machine,
versioned-symbol and ISA checks remain enabled. Save the raw wheel and audit logs. Do not strip. The wrapper compiles with `-O3 -g -DNDEBUG`, without added
LTO, and links with `--disable-new-dtags` to keep `$ORIGIN/.libs` as DT_RPATH.
The official ORT libraries and their notices ship with it. PEP639 license-file
metadata declares all included upstream licenses; no project license is inferred.

For aarch64, pkg uses the robot's existing GCC/sysroot and target CPython3.10
headers/configuration. Confirm those files and SOABI first; do not use host
headers or relabel an x86 binary. `Python_INCLUDE_DIR` selects target headers;
the robot toolchain's `PYTHON_SOABI` is applied after FindPython. Actual ARM
compilation, machine/ABI audit and target execution are separate results.

Place only the two repaired SDK wheels in the release wheelhouse, then run:

```sh
python demo_onnx/tools/wheels.py check-release --wheelhouse <audited-wheelhouse> --report <new-report.json>
```

Missing either architecture, generic Linux tags, incorrect target ABI, missing
license files, stripped debug symbols or extension RUNPATH fail this release
check. It does not execute foreign binaries or establish device acceptance.

## Inference behavior

The policy example owns model construction and supplies the original thread
counts and IO contract. Model loading and inference errors remain Python
exceptions; there is no policy normalization, clipping, history or transport
in the inference backend. Customer code uses the documented SDK and examples,
not the private extension interface. The tests below exercise that interface
internally, including dtype/shape validation and output ownership.

## Tests and parity

```sh
python -m pip install '<audited-sdk-wheel>[teleop-demo]' -r demo_onnx/requirements-test.txt
python -m unittest discover -s demo_onnx/tests -v
```

Tests execute real small ONNX models, including non-tensor IO in subprocesses,
dtype descriptors, unknown rank, zero threads and a hostile LD_LIBRARY_PATH.
The ONNX authoring package is test-only. The oracle lives in a separate
environment with `requirements-oracle.txt` (Python ORT1.20.1). Keep the original
finite float32 numeric exact comparison, signed zeros equal, with no epsilon.
Teleop intentionally changes from Python ORT1.23.2 to C++ ORT1.20.1 to match the
robot; the archived cross-version comparison differed in82/120 outputs. That
historical result is retained, not used as a gate or hidden with a tolerance.
Host inference and packaging results do not establish ARM execution or HIL.

## BodyCmd native encoding

The same SDK wheel also contains `locomotion_aorta._body_cmd_native`, an
independent extension without an ONNX Runtime dependency. Python keeps command
validation, population, ownership, sequence assignment and returned DTOs. The
extension copies inputs while holding the GIL, then releases it while building
the complete FlatBuffer. The pinned Aorta publisher retains its Core clock,
header counter/node policy, schema and QoS; sending uses `publish_bytes`.
There is no automatic Python-encoding fallback.

`cpp/body_cmd_vendor/provenance.json` records the exact generated BODY2 header
and FlatBuffers 25.9.23 header closure/license. `tests/test_native_body_cmd.py`
compares complete buffers with the generated Python builder, including default
elision, explicit action, integer boundaries and fixed31 population semantics.
Installed-wheel validation must run that suite with the native codec present;
a source-only skip is not native acceptance.

Set `LOCOMOTION_AORTA_PUBLISH_TIMING=1` before constructing the controller to
capture per-thread publish diagnostics. Velocity/WBC cycle logs include them
as `publish_timing`, associated with the returned session/sequence. Durations
use monotonic nanoseconds and leave existing S5/S6 timestamps unchanged.
Controller wait, validation, preparation, header/Core-clock call, marshaling,
native encoding, encoding GIL reacquisition and raw-send call are separate.
Nested call-wall durations overlap inner stages and must not be added twice.
The raw-send call wall includes ctypes/GIL reacquisition; it is not pure Core,
network or physical-output latency. Initial GIL wait before entering Python is
unobserved. Profiling is disabled by default and does no file IO inside publish.
