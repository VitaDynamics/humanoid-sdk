"""Packaging negative controls: raw Linux tags and incomplete release sets."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile

TOOL = Path(__file__).resolve().parents[1] / "tools/wheels.py"
spec = importlib.util.spec_from_file_location("native_wheel_tool", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReleaseGateTest(unittest.TestCase):
    def test_generic_linux_wheel_is_not_a_release_asset(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "locomotion_aorta_demo_onnx-0.2.0-cp310-cp310-linux_x86_64.whl"
            with zipfile.ZipFile(path, "w") as wheel:
                wheel.writestr("locomotion_aorta_demo_onnx-0.2.0.dist-info/WHEEL",
                               "Wheel-Version: 1.0\nTag: cp310-cp310-linux_x86_64\n")
            with self.assertRaisesRegex(ValueError, "audited cp310"):
                module.inspect(path)

    def test_release_cannot_pass_without_both_architectures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = root / "report.json"
            result = subprocess.run([sys.executable, str(TOOL), "check-release", "--wheelhouse",
                                     str(root), "--report", str(report)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            data = json.loads(report.read_text())
            self.assertEqual(data["result"], "FAIL")
            self.assertEqual(set(data["missing_arches"]), {"x86_64", "aarch64"})


if __name__ == "__main__":
    unittest.main()
