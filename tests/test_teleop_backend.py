"""Adapter checks run without optional demo dependencies or a robot."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest import mock

from examples.teleop_policy import Model

ROOT = Path(__file__).resolve().parents[1]


def session():
    return NS(inputs=[NS(name="obs", shape=[1, 492], dtype="tensor(float)")],
              outputs=[NS(name="action", shape=[1, 12], dtype="tensor(float)")],
              metadata={"history_length": "6", "clip_actions": "20"})


class TeleopNativeBackendTest(unittest.TestCase):
    def test_uses_native_session_with_original_thread_counts_and_metadata(self):
        expected = session()
        constructor = mock.Mock(return_value=expected)
        with mock.patch.dict(sys.modules, {"locomotion_aorta._onnx": NS(Session=constructor)}):
            model = Model("delivered.onnx")
        constructor.assert_called_once_with("delivered.onnx", intra_threads=1, inter_threads=1)
        self.assertIs(model.session, expected)
        self.assertEqual((model.input_name, model.output_name), ("obs", "action"))
        self.assertEqual(model.metadata, expected.metadata)

    def test_rejects_incompatible_native_io_shapes_and_counts(self):
        for side, shape in [("inputs", [1, 491]), ("outputs", [1, 13])]:
            value = session()
            getattr(value, side)[0].shape = shape
            with self.subTest(side=side), mock.patch.dict(sys.modules, {
                    "locomotion_aorta._onnx": NS(Session=lambda *a, **k: value)}):
                with self.assertRaisesRegex(ValueError, "expected one"):
                    Model("delivered.onnx")
        for side in ["inputs", "outputs"]:
            value = session()
            setattr(value, side, [])
            with self.subTest(side=side), mock.patch.dict(sys.modules, {
                    "locomotion_aorta._onnx": NS(Session=lambda *a, **k: value)}):
                with self.assertRaisesRegex(ValueError, "expected one"):
                    Model("delivered.onnx")

    def test_rejects_nonfloat_native_io(self):
        for side in ["inputs", "outputs"]:
            value = session()
            getattr(value, side)[0].dtype = "tensor(double)"
            with self.subTest(side=side), mock.patch.dict(sys.modules, {
                    "locomotion_aorta._onnx": NS(Session=lambda *a, **k: value)}):
                with self.assertRaisesRegex(ValueError, "float32"):
                    Model("delivered.onnx")

    def test_native_load_error_is_not_hidden_by_another_backend(self):
        constructor = mock.Mock(side_effect=RuntimeError("native load failure"))
        with mock.patch.dict(sys.modules, {"locomotion_aorta._onnx": NS(Session=constructor)}):
            with self.assertRaisesRegex(RuntimeError, "native load failure"):
                Model("bad.onnx")
        constructor.assert_called_once()

    def test_unsupported_python_reports_the_supported_demo_environment(self):
        with mock.patch("sys.version_info", (3, 11)):
            with self.assertRaisesRegex(RuntimeError, "CPython 3.10"):
                Model("delivered.onnx")

    def test_unsupported_architecture_reports_the_supported_demo_environment(self):
        with mock.patch("platform.machine", return_value="s390x"):
            with self.assertRaisesRegex(RuntimeError, "Linux x86_64/aarch64"):
                Model("delivered.onnx")

    def test_missing_native_dependency_does_not_fall_back(self):
        with mock.patch.dict(sys.modules, {"locomotion_aorta._onnx": None}):
            with self.assertRaises(ModuleNotFoundError):
                Model("delivered.onnx")

    def test_core_import_and_teleop_help_do_not_import_optional_backends(self):
        script = """import runpy, sys
class Block:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'locomotion_aorta._onnx' or fullname.startswith('locomotion_aorta._onnx.') or fullname.split('.')[0] in {'onnxruntime', 'pygame'}:
            raise ImportError('optional dependency imported: ' + fullname)
sys.meta_path.insert(0, Block())
import locomotion_aorta
from examples import teleop_policy
sys.argv = ['teleop_policy.py', '--help']
runpy.run_path('examples/teleop_policy.py', run_name='__main__')
"""
        result = subprocess.run([sys.executable, "-c", script], cwd=ROOT,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("--dry-run", result.stdout)


if __name__ == "__main__":
    unittest.main()
