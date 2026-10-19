# Optional demo ONNX backend

This package owns CPU model loading and inference for SDK demos. It is separate
from the `locomotion-aorta` core distribution. SDK extras and demo migration are
not connected yet. No Python ONNX Runtime is imported or used as a fallback.

## Build a wheel

Use Python 3.10+ and CMake 3.20+ on Linux x86_64 or aarch64. The package builds
only the pybind11 wrapper; Microsoft supplies the ORT 1.20.1 CPU runtime.
Download the matching public archive from `ort-archives.json`, or reuse an
existing copy. CMake checks its exact SHA before extraction; there is no
implicit download, dependency on a robot filesystem, or private index.

```sh
python -m pip install build scikit-build-core==0.10.7 pybind11==2.13.6 ninja==1.11.1.3
CMAKE_ARGS='-DORT_ARCHIVE=/absolute/path/to/official-archive.tgz' \
  python -m build --wheel --no-isolation demo_onnx
python -m pip install demo_onnx/dist/locomotion_aorta_demo_onnx-*.whl
```

The wheel carries the real ORT SONAME file, provider support library and upstream
notices. Its extension uses `$ORIGIN/.libs`; consumers do not need `/app` libraries
or a compiler. `build-info.json` identifies the official archive SHA, target CPU,
compiler and Python ABI. Platform/glibc compatibility must be audited for the
actual release wheel. This source does not claim an ARM wheel has been tested.
The version `0.2.0` is an unreleased candidate, not a published SDK dependency.

## Interface

```python
from locomotion_aorta_demo_onnx import ORT_VERSION, Session
session = Session(model_path, intra_threads=1, inter_threads=1)
metadata = session.metadata  # a copy of custom metadata; no renamed contract keys
input_info, output_info = session.inputs[0], session.outputs[0]
output = session.run(input_info.name, output_info.name, float32_array)
```

Thread counts are mandatory keyword arguments. Demos must pass their existing
values (teleop/velocity 1/1; WBC configured intra-op count and inter-op 1).
Other session options remain ORT defaults. IO descriptions expose `name`,
`shape` (dynamic dimensions as -1) and `dtype`, e.g. `tensor(float)`.
`run` supports one selected float32 input and output, preserving the input rank.
It copies strided or dense input into an owned snapshot before releasing the
GIL for ORT. Each result is a fresh owning NumPy array; another call cannot
overwrite it. No policy normalization, clipping, history or transport is here.

Wrong names, tensor shape/dtype or nonpositive thread counts raise `ValueError`;
missing required keywords or a non-array argument raise `TypeError`. Model-load
and ORT execution failures raise `RuntimeError` with context. A runtime version
other than 1.20.1 raises `ImportError`; it is never silently substituted.

## Tests and model parity

```sh
python -m pip install -r demo_onnx/requirements-test.txt
python -m unittest discover -s demo_onnx/tests -v
```

Tests create small real ONNX models in temporary directories and execute the
installed native wheel. They do not skip absent dependencies. The ONNX authoring
package is test-only. A Python ORT oracle is not a runtime dependency.

Real-model comparison uses the same model SHA, float32 inputs and thread counts
in separate processes: the test-only Python ORT 1.20.1 oracle versus this
ORT 1.20.1 backend. Install the oracle in its own environment using
`demo_onnx/requirements-oracle.txt`; it is not a demo runtime dependency.
Follow the existing teleop oracle's finite float32 numeric equality (signed
zeros equal), with no epsilon. The earlier Python ORT 1.23.2 cross-version
comparison is retained as historical evidence, not an acceptance gate.
Existing teleop obs/target fixture results alone do not establish inference
parity or HIL.
