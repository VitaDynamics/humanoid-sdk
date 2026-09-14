import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from tools import package_release


ROOT = Path(__file__).resolve().parents[1]


class ReleaseTest(unittest.TestCase):
    def test_pc_profile_is_peer_and_contains_only_deployment_placeholders(self):
        profile = json.loads((ROOT / "config/pc_session_peer.json5").read_text())
        self.assertEqual(profile["mode"], "peer")
        self.assertEqual(profile["namespace"], "REPLACE_WITH_ROBOT_NAMESPACE")
        self.assertEqual(profile["transport"]["auth"]["usrpwd"], {
            "user": "REPLACE_WITH_DEPLOYMENT_USER",
            "password": "REPLACE_WITH_DEPLOYMENT_PASSWORD",
            "dictionary_file": "/REPLACE_WITH_ABSOLUTE_PATH/peer-auth.txt",
        })
        self.assertTrue(profile["scouting"]["gossip"]["enabled"])
        self.assertFalse(profile["scouting"]["multicast"]["enabled"])
        self.assertEqual(profile["scouting"]["gossip"]["autoconnect"]["peer"], ["peer"])
        self.assertEqual(profile["connect"]["endpoints"], ["tcp/192.168.125.2:7447"])
        self.assertEqual(profile["listen"]["endpoints"], ["tcp/192.168.125.3:0"])
        self.assertNotIn("region_name", profile)
        self.assertNotIn("gateway", profile)

    def test_bundle_includes_only_public_profile_with_manifest_digest(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "sdk"
            root.mkdir()
            for name in ("README.md", "AGENTS.md", "compatibility.json"):
                shutil.copyfile(ROOT / name, root / name)
            (root / "CLAUDE.md").symlink_to("AGENTS.md")
            for name in ("examples", "python/locomotion_aorta", "config", "dist", "wheels"):
                (root / name).mkdir(parents=True)
            source = ROOT / "config/pc_session_peer.json5"
            if source.exists():
                shutil.copyfile(source, root / "config/pc_session_peer.json5")
            (root / "config/private-auth.txt").write_text("private fixture, must not ship")
            compatibility = json.loads((root / "compatibility.json").read_text())
            for name in compatibility["wheel_sha256"]:
                payload = name.encode()
                (root / "wheels" / name).write_bytes(payload)
                compatibility["wheel_sha256"][name] = hashlib.sha256(payload).hexdigest()
            (root / "compatibility.json").write_text(json.dumps(compatibility))
            (root / "dist/locomotion_aorta-0.1.0-py3-none-any.whl").write_bytes(b"sdk fixture")
            output = Path(temporary) / "bundle"
            with mock.patch.object(package_release, "ROOT", root), mock.patch.object(
                package_release.subprocess, "check_output", side_effect=["fixture-head\n", ""]
            ):
                package_release.package([root / "wheels"], output)
            bundled = output / "config/pc_session_peer.json5"
            self.assertTrue(bundled.is_file())
            self.assertEqual(bundled.read_bytes(), source.read_bytes())
            self.assertEqual([p.name for p in (output / "config").iterdir()], [source.name])
            manifest = json.loads((output / "MANIFEST.json").read_text())
            self.assertEqual(manifest["pc_session_profile"], "config/pc_session_peer.json5")
            self.assertEqual(manifest["files"]["config/pc_session_peer.json5"],
                             hashlib.sha256(bundled.read_bytes()).hexdigest())
            private_profile = json.loads(source.read_text())
            private_profile["transport"]["auth"]["usrpwd"]["password"] = "test-only-value"
            (root / "config/pc_session_peer.json5").write_text(json.dumps(private_profile))
            refused = Path(temporary) / "private-bundle"
            with mock.patch.object(package_release, "ROOT", root):
                with self.assertRaisesRegex(ValueError, "deployment placeholders"):
                    package_release.package([root / "wheels"], refused)
            self.assertFalse(refused.exists())

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
