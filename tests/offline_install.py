"""Verify a built bundle in a fresh venv without an index or robot motion."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile


class OfflineInstallTest(unittest.TestCase):
    def test_complete_bundle_installs_without_index(self):
        bundle = Path(sys.argv[1]).resolve()
        manifest = json.loads((bundle / "MANIFEST.json").read_text())
        actual_files = {
            path.relative_to(bundle).as_posix()
            for path in bundle.rglob("*")
            if path.is_file() and path.name != "MANIFEST.json"
        }
        self.assertEqual(set(manifest["files"]), actual_files)
        for relative, expected in manifest["files"].items():
            self.assertEqual(
                hashlib.sha256((bundle / relative).read_bytes()).hexdigest(), expected
            )
        self.assertTrue((bundle / "CLAUDE.md").is_symlink())
        self.assertEqual(str((bundle / "CLAUDE.md").readlink()), "AGENTS.md")
        sdk_wheel = bundle / "wheelhouse" / (
            f"locomotion_aorta-{manifest['sdk_version']}-py3-none-any.whl"
        )
        with zipfile.ZipFile(sdk_wheel) as wheel:
            for path in (bundle / "source/locomotion_aorta").glob("*.py"):
                self.assertEqual(
                    wheel.read(f"locomotion_aorta/{path.name}"), path.read_bytes()
                )
        with tempfile.TemporaryDirectory() as temporary:
            venv = Path(temporary) / "venv"
            subprocess.run([sys.executable, "-m", "venv", str(venv)], check=True)
            python = str(venv / "bin/python")
            subprocess.run(
                [python, "-I", "-m", "pip", "install", "--no-index",
                 "--find-links", str(bundle / "wheelhouse"),
                 f"locomotion-aorta=={manifest['sdk_version']}"], check=True,
            )
            subprocess.run([python, "-I", "-m", "pip", "check"], check=True)
            subprocess.run(
                [python, "-I", "-c",
                 "import aorta, flatbuffers, locomotion_aorta; "
                 "import locomotion_sdk.ControlStatus, lowlevel.LowCmd; "
                 "from locomotion_aorta.transport import _load_aorta_bindings; "
                 "_load_aorta_bindings(); print('real bindings OK')"], check=True,
            )
            for example in ("lowstate_subscriber.py", "external_control.py"):
                subprocess.run(
                    [python, "-I", str(bundle / "examples" / example), "--help"],
                    check=True,
                )
            denied = subprocess.run(
                [python, "-I", str(bundle / "examples/external_control.py")],
                capture_output=True, text=True,
            )
            self.assertEqual(denied.returncode, 2)
            self.assertIn("--confirm-motion", denied.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python tests/offline_install.py BUNDLE_DIRECTORY")
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(OfflineInstallTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
