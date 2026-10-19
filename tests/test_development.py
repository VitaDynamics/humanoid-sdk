"""The developer test command imports the working tree, not an installed SDK."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class DevelopmentTest(unittest.TestCase):
    def test_make_test_imports_the_source_package(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            shutil.copy2(ROOT / "Makefile", root / "Makefile")
            package = root / "python/locomotion_aorta"
            package.mkdir(parents=True)
            (package / "__init__.py").write_text("source_marker = 'working-tree'\n")
            tests = root / "tests"
            tests.mkdir()
            (tests / "test_origin.py").write_text(
                "import unittest, locomotion_aorta\n"
                "class Origin(unittest.TestCase):\n"
                "    def test_origin(self):\n"
                "        self.assertEqual(locomotion_aorta.source_marker, 'working-tree')\n")
            result = subprocess.run(["make", "test", "PYTHON=" + sys.executable],
                                    cwd=root, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            examples = root / "examples"
            examples.mkdir()
            (examples / "body_state_subscriber.py").write_text(
                "import locomotion_aorta\n"
                "assert locomotion_aorta.source_marker == 'working-tree'\n")
            # The other check prerequisites are exercised by the actual full check;
            # isolate its source-importing recipe here without installing an SDK.
            result = subprocess.run(
                ["make", "-f", "Makefile", "-f", "-", "check", "PYTHON=" + sys.executable],
                input="check:; $(PYTHON) examples/body_state_subscriber.py\n",
                cwd=root, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
