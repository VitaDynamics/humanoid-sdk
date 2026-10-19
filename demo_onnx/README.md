# Optional demo ONNX backend

SDK 0.2.0 embeds the pybind11/Microsoft ORT 1.20.1 CPU backend directly in
its only distribution, `locomotion-aorta`. There is no standalone backend
package and no pure Python SDK wheel for this release. The private
`locomotion_aorta._onnx` subtree is imported only when a demo constructs a model;
SDK core imports do not load it. The `teleop-demo` extra contains NumPy and
pygame; there is no Python ORT fallback.

SDK 0.2.0 supports **CPython 3.10, Linux x86_64 and aarch64, Ubuntu 22.04+**
(glibc 2.35 / GLIBCXX 3.4.29+). Both audited architecture wheels are required
before release. Version 0.2.1 is planned to restore general platform support
through platform and pure Python wheels under the same SDK package name.

## Local build and release audit

Use CMake 3.20+ and a C++17 compiler. Obtain the architecture-matching official
CPU archive from `ort-archives.json`; CMake verifies its SHA before extraction.
There are no implicit downloads or dependencies on a robot's `/app` filesystem.

```sh
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

## Interface and errors

```python
from locomotion_aorta._onnx import Session
session = Session(model_path, intra_threads=1, inter_threads=1)
output = session.run(session.inputs[0].name, session.outputs[0].name, float32_array)
```

Thread counts are required keywords and passed to ORT unchanged; 0 uses ORT's
default. Existing demos retain their original values. IO descriptors expose
name, shape and dtype; metadata is copied. Non-tensor IO raises ValueError.
Unknown rank is delegated to ORT, not treated as a scalar-only input. Float32
dtypes are compared by value; nonnative byte order is not implicitly cast.
Input data is snapshotted under the GIL, Run releases it, and each output owns
its storage. No policy normalization, clipping, history or transport is here.

The private test interface below is not imported from the SDK root.
Wrong names, known shape/dtype and non-tensor IO raise ValueError; missing
keywords or non-array arguments raise TypeError. ORT load/run failures become
RuntimeError with context. The loaded runtime must report version1.20.1.

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
