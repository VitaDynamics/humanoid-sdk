"""Verify a built bundle in a fresh venv without an index or robot motion."""

import hashlib
import json
import platform
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
        self.assertEqual(manifest["pc_session_profile"], "config/pc_session_peer.json5")
        profile = json.loads((bundle / manifest["pc_session_profile"]).read_text())
        self.assertEqual(profile["mode"], "peer")
        self.assertEqual(profile["namespace"], "REPLACE_WITH_ROBOT_NAMESPACE")
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
            f"locomotion_aorta-{manifest['sdk_version']}-cp310-cp310-manylinux_2_35_{platform.machine()}.whl"
        )
        with zipfile.ZipFile(sdk_wheel) as wheel:
            for path in (bundle / "source/locomotion_aorta").rglob("*.py"):
                relative = path.relative_to(bundle / "source")
                self.assertEqual(wheel.read(relative.as_posix()), path.read_bytes())
            self.assertIn("locomotion_aorta/_onnx/.libs/libonnxruntime.so.1", wheel.namelist())
            self.assertTrue(any(name.startswith("locomotion_aorta/_onnx/_native.cpython-310-")
                                and name.endswith(".so") for name in wheel.namelist()))
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
                 "import aorta, flatbuffers, locomotion_aorta, sys; "
                 "assert not any(n.startswith('locomotion_aorta._onnx') for n in sys.modules); "
                 "assert 'onnxruntime' not in sys.modules; "
                 "import importlib.metadata as metadata; "
                 "import locomotion_sdk.ControlStatus, lowlevel.BodyCmd, lowlevel.BodyState; "
                 "from locomotion_aorta.transport import _load_aorta_bindings; "
                 "_load_aorta_bindings(); "
                 "from locomotion_aorta._hand_codec import load_bindings; load_bindings(); "
                 f"assert metadata.version('vbot-humanoid-msgs') == {manifest['humanoid_msgs_version']!r}; "
                 f"assert aorta.load_library().aorta_abi_version() == {manifest['aorta_abi_version']}; "
                 "print('real bindings and pinned humanoid bindings/native ABI OK')"], check=True,
            )
            for retired in ("brainco_hand_control.py", "gripper_external_lifecycle.py", "lowstate_subscriber.py"):
                self.assertFalse((bundle / "examples" / retired).exists())
            for example in ("body_state_subscriber.py", "external_control.py",
                            "brainco_hand_external.py", "brainco_capture.py", "teleop_policy.py",
                            "gripper_external.py"):
                subprocess.run(
                    [python, "-I", str(bundle / "examples" / example), "--help"],
                    check=True,
                )
            denied = subprocess.run(
                [python, "-I", str(bundle / "examples/external_control/external_control.py")],
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
