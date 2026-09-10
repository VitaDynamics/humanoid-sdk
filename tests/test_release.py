import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ReleaseTest(unittest.TestCase):
    def test_missing_wheels_fail_before_output_is_created(self):
        script = ROOT / "tools/package_release.py"
        self.assertTrue(script.is_file(), "offline release builder is missing")
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "release"
            result = subprocess.run(
                [sys.executable, str(script), "--wheelhouse", temporary,
                 "--output", str(output)],
                capture_output=True, text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("missing wheel", result.stderr)
            self.assertFalse(output.exists())

    def test_corrupted_dependency_is_rejected_before_output(self):
        script = ROOT / "tools/package_release.py"
        self.assertTrue(script.is_file(), "offline release builder is missing")
        manifest = json.loads((ROOT / "compatibility.json").read_text())
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            for name in manifest["wheel_sha256"]:
                (directory / name).write_bytes(b"corrupt")
            result = subprocess.run(
                [sys.executable, str(script), "--wheelhouse", temporary,
                 "--output", str(directory / "release")],
                capture_output=True, text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("SHA-256 mismatch", result.stderr)
            self.assertFalse((directory / "release").exists())
