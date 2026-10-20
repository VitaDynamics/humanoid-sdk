import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from tools import fetch_dependencies


ROOT = Path(__file__).resolve().parents[1]


class FetchDependenciesTest(unittest.TestCase):
    def setUp(self):
        self.manifest = json.loads((ROOT / "compatibility.json").read_text())

    def fetch(self, root, manifest, arch="x86_64", download=None):
        path = root / "compatibility.json"
        path.write_text(json.dumps(manifest))
        with mock.patch.object(fetch_dependencies, "MANIFEST", path), mock.patch.object(
            fetch_dependencies.subprocess, "run", side_effect=download
        ) as run:
            result = fetch_dependencies.fetch(root / "wheels", arch)
            return result, run.call_args_list

    def fixture(self):
        manifest = copy.deepcopy(self.manifest)
        for name in manifest["wheel_sha256"]:
            manifest["wheel_sha256"][name] = hashlib.sha256(name.encode()).hexdigest()
        return manifest

    @staticmethod
    def download(command, **kwargs):
        output = Path(command[command.index("--dir") + 1])
        name = command[command.index("--pattern") + 1]
        (output / name).write_bytes(name.encode())

    def test_official_release_identity(self):
        self.assertEqual(self.manifest["dependency_release_tag"], "v0.1.0-rc.4")
        self.assertEqual(self.manifest["dependency_release_repository"], "VitaDynamics/humanoid-sdk")
        self.assertEqual(self.manifest["wheel_sha256"]["vbot_humanoid_msgs-0.1.3-py3-none-any.whl"],
                         "cc21b3198248f91e433d9665d0284de66a7db765e952e2abfb5afcb054ec2a5f")

    def test_each_architecture_downloads_four_exact_assets_from_the_sdk_release(self):
        for arch in ("x86_64", "aarch64"):
            with self.subTest(arch=arch), tempfile.TemporaryDirectory() as directory:
                output, calls = self.fetch(Path(directory), self.fixture(), arch, self.download)
                self.assertEqual(len(list(output.iterdir())), 4)
                self.assertEqual(len(calls), 4)
                for call in calls:
                    command = call.args[0]
                    name = command[-1]
                    self.assertEqual(command[3], "v0.1.0-rc.4")
                    self.assertEqual(command[5], "VitaDynamics/humanoid-sdk")
                    self.assertNotIn("env", call.kwargs)
                    self.assertTrue(call.kwargs["check"])
                    if name.startswith("aorta_sdk-"):
                        self.assertTrue(name.endswith(f"-linux_{arch}.whl"))

    def test_missing_or_invalid_tags_fail_before_any_download(self):
        for key in ("dependency_release_tag",):
            for value in (None, "", " \t", 0):
                with self.subTest(key=key, value=value), tempfile.TemporaryDirectory() as directory:
                    manifest = copy.deepcopy(self.manifest)
                    manifest[key] = value
                    with mock.patch.object(fetch_dependencies.subprocess, "run") as run:
                        with self.assertRaisesRegex(ValueError, key + " must be a nonempty string"):
                            self.fetch(Path(directory), manifest)
                        run.assert_not_called()
                    self.assertFalse((Path(directory) / "wheels").exists())

    def test_incomplete_wheelhouse_is_rejected_before_download(self):
        manifest = self.fixture()
        del manifest["wheel_sha256"]["vbot_humanoid_msgs-0.1.3-py3-none-any.whl"]
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "incomplete pinned wheelhouse"):
                self.fetch(Path(directory), manifest)
            self.assertFalse((Path(directory) / "wheels").exists())

    def test_missing_release_repository_fails_before_any_download(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "compatibility.json"
            manifest = copy.deepcopy(self.manifest)
            del manifest["dependency_release_repository"]
            path.write_text(json.dumps(manifest))
            with mock.patch.object(fetch_dependencies, "MANIFEST", path), mock.patch.object(
                fetch_dependencies.subprocess, "run"
            ) as run:
                with self.assertRaisesRegex(ValueError, "dependency_release_repository must be a nonempty string"):
                    fetch_dependencies.fetch(Path(directory) / "wheels", "x86_64")
                run.assert_not_called()

    def test_failed_download_is_not_retried_or_substituted(self):
        failed = subprocess.CalledProcessError(1, ["gh", "release", "download"])
        calls = []
        with tempfile.TemporaryDirectory() as directory:
            def download(command, **kwargs):
                calls.append(command)
                raise failed
            with self.assertRaisesRegex(RuntimeError, "humanoid-sdk#dependency-release-convention") as error:
                self.fetch(Path(directory), self.fixture(), download=download)
            self.assertIs(error.exception.__cause__, failed)
            self.assertIn("v0.1.0-rc.4", str(error.exception))
            self.assertIn(calls[0][-1], str(error.exception))
            self.assertEqual(len(calls), 1)

    def test_corrupt_asset_fails_and_is_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                self.fetch(root, self.manifest, download=self.download)
            self.assertTrue(any((root / "wheels").iterdir()))


if __name__ == "__main__":
    unittest.main()
