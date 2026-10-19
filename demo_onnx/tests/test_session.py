"""Real native-wheel tests; dependencies are required, never skipped."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

from locomotion_aorta_demo_onnx import ORT_VERSION, Session


def write_model(path, *, dynamic=False, dtype=TensorProto.FLOAT, two_inputs=False):
    shape = ["batch" if dynamic else 1, 3]
    inputs = [helper.make_tensor_value_info("obs", dtype, shape)]
    if two_inputs:
        inputs.append(helper.make_tensor_value_info("extra", dtype, shape))
        other = "extra"
        initializers = []
    else:
        other = "bias"
        values = np.array([0.5, 1.0, -2.0], dtype=np.float16 if dtype == TensorProto.FLOAT16 else np.float32)
        initializers = [numpy_helper.from_array(values, other)]
    graph = helper.make_graph([helper.make_node("Add", ["obs", other], ["action"])],
                              "native-fixture", inputs,
                              [helper.make_tensor_value_info("action", dtype, shape)],
                              initializers)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 10
    helper.set_model_props(model, {"contract": "unchanged", "history_length": "1"})
    onnx.checker.check_model(model)
    onnx.save(model, path)


class NativeSessionTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="demo-onnx-test-")
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "model.onnx"
        write_model(self.path)
        self.session = Session(str(self.path), intra_threads=1, inter_threads=1)
        self.input = np.array([[0.0, 2.0, 4.0]], dtype=np.float32)

    def test_metadata_and_tensor_descriptions_preserve_contract(self):
        self.assertEqual(ORT_VERSION, "1.20.1")
        self.assertEqual(self.session.metadata, {"contract": "unchanged", "history_length": "1"})
        for tensor, name in [(self.session.inputs[0], "obs"), (self.session.outputs[0], "action")]:
            self.assertEqual((tensor.name, tensor.shape, tensor.dtype), (name, [1, 3], "tensor(float)"))
        self.assertEqual((self.session.intra_threads, self.session.inter_threads), (1, 1))

    def test_real_inference_exact_values(self):
        result = self.session.run("obs", "action", self.input)
        np.testing.assert_array_equal(result, [[0.5, 3.0, 2.0]])
        self.assertEqual(result.dtype, np.dtype("float32"))
        self.assertEqual(result.shape, (1, 3))

    def test_strided_input_is_copied_without_reordering(self):
        values = np.arange(6, dtype=np.float32).reshape(1, 6)[:, ::2]
        self.assertFalse(values.flags.c_contiguous)
        np.testing.assert_array_equal(self.session.run("obs", "action", values), [[0.5, 3.0, 2.0]])

    def test_readonly_input_and_owned_results(self):
        self.input.flags.writeable = False
        first = self.session.run("obs", "action", self.input)
        saved = first.copy()
        second = self.session.run("obs", "action", self.input + 8)
        self.assertTrue(first.flags.owndata)
        np.testing.assert_array_equal(first, saved)
        second[:] = 123
        np.testing.assert_array_equal(self.session.run("obs", "action", self.input), saved)
        np.testing.assert_array_equal(self.input, [[0.0, 2.0, 4.0]])

    def test_metadata_and_descriptions_do_not_expose_mutable_session_state(self):
        metadata = self.session.metadata
        metadata["contract"] = "bad"
        shape = self.session.inputs[0].shape
        shape[0] = 99
        self.assertEqual(self.session.metadata["contract"], "unchanged")
        self.assertEqual(self.session.inputs[0].shape, [1, 3])
        with self.assertRaises(AttributeError):
            self.session.inputs[0].name = "bad"

    def test_wrong_names_are_value_errors(self):
        for names in [("missing", "action"), ("obs", "missing")]:
            with self.subTest(names=names), self.assertRaisesRegex(ValueError, "unknown"):
                self.session.run(*names, self.input)

    def test_wrong_rank_shape_and_dtype_are_value_errors(self):
        for array in [self.input[0], np.zeros((1, 4), np.float32), self.input.astype(np.float64)]:
            with self.subTest(shape=array.shape, dtype=array.dtype), self.assertRaises(ValueError):
                self.session.run("obs", "action", array)

    def test_array_argument_does_not_implicitly_convert(self):
        with self.assertRaises(TypeError):
            self.session.run("obs", "action", [[0, 2, 4]])

    def test_thread_options_are_required_keywords(self):
        for args, kwargs in [((str(self.path),), {}), ((str(self.path), 1, 1), {})]:
            with self.assertRaises(TypeError):
                Session(*args, **kwargs)
        for intra, inter in [(0, 1), (1, 0), (-1, 1)]:
            with self.subTest(intra=intra, inter=inter), self.assertRaises(ValueError):
                Session(str(self.path), intra_threads=intra, inter_threads=inter)

    def test_invalid_model_failure_does_not_poison_another_session(self):
        missing = self.path.with_name("absent.onnx")
        with self.assertRaisesRegex(RuntimeError, "ONNX load.*absent.onnx"):
            Session(str(missing), intra_threads=1, inter_threads=1)
        invalid = self.path.with_name("invalid.onnx")
        invalid.write_bytes(b"not a model")
        with self.assertRaisesRegex(RuntimeError, "ONNX load"):
            Session(str(invalid), intra_threads=1, inter_threads=1)
        np.testing.assert_array_equal(self.session.run("obs", "action", self.input), [[0.5, 3.0, 2.0]])

    def test_dynamic_dimensions_are_resolved_by_each_call(self):
        write_model(self.path, dynamic=True)
        session = Session(str(self.path), intra_threads=1, inter_threads=1)
        self.assertEqual(session.inputs[0].shape, [-1, 3])
        values = np.arange(6, dtype=np.float32).reshape(2, 3)
        np.testing.assert_array_equal(session.run("obs", "action", values), values + [0.5, 1.0, -2.0])

    def test_nonfloat_model_is_described_but_not_implicitly_cast(self):
        write_model(self.path, dtype=TensorProto.FLOAT16)
        session = Session(str(self.path), intra_threads=1, inter_threads=1)
        self.assertEqual(session.inputs[0].dtype, "tensor(float16)")
        with self.assertRaises(ValueError):
            session.run("obs", "action", self.input.astype(np.float16))

    def test_ort_execution_error_retains_call_context(self):
        write_model(self.path, two_inputs=True)
        session = Session(str(self.path), intra_threads=1, inter_threads=1)
        with self.assertRaisesRegex(RuntimeError, "ONNX run 'obs' -> 'action'"):
            session.run("obs", "action", self.input)

    def test_concurrent_runs_have_independent_inputs_and_results(self):
        def infer(i):
            values = self.input + i
            return self.session.run("obs", "action", values)
        with ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(infer, range(20)))
        for i, result in enumerate(results):
            np.testing.assert_array_equal(result, self.input + i + [0.5, 1.0, -2.0])
        results[0][:] = 999
        np.testing.assert_array_equal(results[1], self.input + 1 + [0.5, 1.0, -2.0])

    def test_native_runs_when_python_ort_import_is_blocked(self):
        script = """import sys
class Block:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'onnxruntime' or fullname.startswith('onnxruntime.'):
            raise ImportError('Python ORT must not be used')
sys.meta_path.insert(0, Block())
import numpy as np
from locomotion_aorta_demo_onnx import Session
s = Session(sys.argv[1], intra_threads=1, inter_threads=1)
assert s.run('obs', 'action', np.zeros((1, 3), np.float32)).tolist() == [[0.5, 1.0, -2.0]]
assert 'onnxruntime' not in sys.modules
"""
        result = subprocess.run([sys.executable, "-I", "-c", script, str(self.path)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
