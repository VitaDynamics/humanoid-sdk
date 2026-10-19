"""Real ORT regressions for review findings; no robot or optional fallback."""
import os
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import onnx
from onnx import TensorProto as T, helper, numpy_helper
from locomotion_aorta_demo_onnx import Session
from test_session import write_model


def save(path, kind):
    tensor = helper.make_tensor_value_info("obs", T.FLOAT, [1, 3])
    output = helper.make_tensor_value_info("action", T.FLOAT, [1, 3])
    sequence = helper.make_tensor_sequence_value_info("seq", T.FLOAT, [1, 3])
    optional = helper.make_value_info("maybe", helper.make_optional_type_proto(
        helper.make_tensor_type_proto(T.FLOAT, [1, 3])))
    initializers = []
    imports = [helper.make_opsetid("", 15)]
    if kind == "sequence_output":
        inputs, outputs = [tensor], [output, sequence]
        nodes = [helper.make_node("Identity", ["obs"], ["action"]),
                 helper.make_node("SequenceConstruct", ["obs"], ["seq"])]
    elif kind == "sequence_input":
        inputs, outputs = [sequence], [output]
        initializers = [numpy_helper.from_array(np.array(0, dtype=np.int64), "index")]
        nodes = [helper.make_node("SequenceAt", ["seq", "index"], ["action"])]
    elif kind == "optional_output":
        inputs, outputs = [tensor], [optional]
        nodes = [helper.make_node("Optional", ["obs"], ["maybe"])]
    elif kind == "optional_input":
        inputs, outputs = [optional], [output]
        nodes = [helper.make_node("OptionalGetElement", ["maybe"], ["action"])]
    elif kind == "map_input":
        mapping = helper.make_value_info("features", helper.make_map_type_proto(
            T.STRING, helper.make_tensor_type_proto(T.FLOAT, [])))
        inputs, outputs = [mapping], [output]
        nodes = [helper.make_node("DictVectorizer", ["features"], ["action"],
                                  domain="ai.onnx.ml", string_vocabulary=["a", "b", "c"])]
        imports.append(helper.make_opsetid("ai.onnx.ml", 1))
    else:
        raise ValueError(kind)
    graph = helper.make_graph(nodes, kind, inputs, outputs, initializers)
    model = helper.make_model(graph, opset_imports=imports)
    model.ir_version = 10
    onnx.checker.check_model(model)
    onnx.save(model, path)


class ReviewRegressionTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="native-review-")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.path = self.root / "model.onnx"

    def test_non_tensor_io_raises_instead_of_crashing_the_process(self):
        script = """import resource, sys
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
from locomotion_aorta_demo_onnx import Session
try:
    Session(sys.argv[1], intra_threads=1, inter_threads=1)
except ValueError as error:
    assert sys.argv[2] in str(error) and 'not a tensor' in str(error), str(error)
else:
    raise AssertionError('non-tensor IO unexpectedly accepted')
"""
        for kind in ["sequence_input", "sequence_output", "optional_input", "optional_output", "map_input"]:
            with self.subTest(kind=kind):
                save(self.path, kind)
                side = "input" if kind.endswith("input") else "output"
                result = subprocess.run([sys.executable, "-I", "-c", script, str(self.path), side],
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_float32_dtype_is_compared_by_value(self):
        write_model(self.path)
        session = Session(str(self.path), intra_threads=1, inter_threads=1)
        values = np.array([[0, 2, 4]], dtype=np.float32)
        variants = [pickle.loads(pickle.dumps(values)),
                    values.view(np.dtype(np.float32, metadata={"source": "ipc"}))]
        for array in variants:
            np.testing.assert_array_equal(session.run("obs", "action", array), [[0.5, 3, 2]])
        with self.assertRaises(ValueError):
            session.run("obs", "action", values.astype(">f4"))

    def test_unknown_rank_is_delegated_to_ort(self):
        # ORT accepts a tensor type with no shape; an empty shape from its API
        # must not invent a rank-zero restriction in the wrapper.
        graph = helper.make_graph([helper.make_node("Identity", ["obs"], ["action"])],
                                  "unknown-rank",
                                  [helper.make_tensor_value_info("obs", T.FLOAT, None)],
                                  [helper.make_tensor_value_info("action", T.FLOAT, None)])
        model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
        model.ir_version = 10
        onnx.save(model, self.path)
        session = Session(str(self.path), intra_threads=1, inter_threads=1)
        self.assertEqual(session.inputs[0].shape, [])
        for shape in [(1, 3), (2, 1, 3), ()]:
            values = np.ones(shape, dtype=np.float32)
            np.testing.assert_array_equal(session.run("obs", "action", values), values)

    def test_zero_thread_counts_use_ort_defaults(self):
        write_model(self.path)
        session = Session(str(self.path), intra_threads=0, inter_threads=0)
        self.assertEqual((session.intra_threads, session.inter_threads), (0, 0))
        np.testing.assert_array_equal(session.run("obs", "action", np.zeros((1, 3), np.float32)),
                                      [[0.5, 1, -2]])

    def test_bundled_ort_precedes_ld_library_path(self):
        write_model(self.path)
        decoy = self.root / "decoy"
        decoy.mkdir()
        (decoy / "libonnxruntime.so.1").write_text("not an ELF library")
        script = """import sys
import numpy as np
from locomotion_aorta_demo_onnx import Session
s = Session(sys.argv[1], intra_threads=1, inter_threads=1)
assert s.run('obs', 'action', np.zeros((1, 3), np.float32)).tolist() == [[0.5, 1.0, -2.0]]
"""
        result = subprocess.run([sys.executable, "-I", "-c", script, str(self.path)],
                                env={**os.environ, "LD_LIBRARY_PATH": str(decoy)},
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
