"""Exercise the pinned mirror, integrity and no-overwrite boundary offline."""
import hashlib
import json
import subprocess
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools import fetch_ort


class FetchOrtTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.payload = b"official archive fixture"
        manifest = self.root / "ort.json"
        manifest.write_text(json.dumps({
            "mirror_release": {"repository": "example/sdk", "tag": "fixed-ort"},
            "x86_64": {"directory": "fixed-ort-x64",
                       "sha256": hashlib.sha256(self.payload).hexdigest()}}))
        self.output = self.root / "download/ort.tgz"
        for patch in (mock.patch.object(fetch_ort, "MANIFEST", manifest),
                      mock.patch.object(fetch_ort.platform, "machine", return_value="x86_64")):
            patch.start()
            self.addCleanup(patch.stop)

    def fetch(self, payload, *, error=None, race=False):
        def download(argv, *, check):
            pending = Path(argv[-1])
            self.assertTrue(check)
            self.assertEqual(argv[:-1], ["gh", "release", "download", "fixed-ort",
                "--repo", "example/sdk", "--pattern", "fixed-ort-x64.tgz", "--output"])
            self.assertNotEqual(pending, self.output)
            self.assertEqual(pending.parent.parent, self.output.parent)
            pending.write_bytes(payload)
            if race:
                self.output.write_bytes(b"other owner")
            if error:
                raise error
        with mock.patch.object(fetch_ort.subprocess, "run", side_effect=download) as get:
            try:
                return fetch_ort.fetch(self.output)
            finally:
                get.assert_called_once()

    def test_downloads_explicit_mirror_and_verifies_the_actual_bytes(self):
        self.assertEqual(self.fetch(self.payload), self.output.resolve())
        self.assertEqual(self.output.read_bytes(), self.payload)
        self.assertEqual(list(self.output.parent.glob(".ort.tgz-*")), [])

    def test_wrong_sha_fails_and_retains_the_failed_download(self):
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            self.fetch(b"corrupt")
        self.assertFalse(self.output.exists())
        retained = list(self.output.parent.glob(".ort.tgz-*/ort.tgz"))
        self.assertEqual(len(retained), 1)
        self.assertEqual(retained[0].read_bytes(), b"corrupt")

    def test_existing_output_is_preserved_without_network(self):
        self.output.parent.mkdir()
        self.output.write_bytes(b"keep")
        with mock.patch.object(fetch_ort.subprocess, "run") as get:
            with self.assertRaises(FileExistsError):
                fetch_ort.fetch(self.output)
            get.assert_not_called()
        self.assertEqual(self.output.read_bytes(), b"keep")

    def test_release_failure_is_not_retried_and_explicit_next_call_can_succeed(self):
        failed = subprocess.CalledProcessError(1, ["gh"])
        with self.assertRaises(subprocess.CalledProcessError) as raised:
            self.fetch(b"partial", error=failed)
        self.assertIs(raised.exception, failed)
        self.assertFalse(self.output.exists())
        retained = list(self.output.parent.glob(".ort.tgz-*/ort.tgz"))
        self.assertEqual(len(retained), 1)
        self.fetch(self.payload)
        self.assertEqual(self.output.read_bytes(), self.payload)
        self.assertEqual(retained[0].read_bytes(), b"partial")

    def test_output_created_during_download_is_never_overwritten(self):
        with self.assertRaises(FileExistsError):
            self.fetch(self.payload, race=True)
        self.assertEqual(self.output.read_bytes(), b"other owner")
        retained = list(self.output.parent.glob(".ort.tgz-*/ort.tgz"))
        self.assertEqual(len(retained), 1)
        self.assertEqual(retained[0].read_bytes(), self.payload)
