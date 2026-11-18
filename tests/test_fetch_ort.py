"""Exercise the real archive supply path without network or compilation."""
import hashlib
import io
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
        self.manifest = self.root / "ort.json"
        self.manifest.write_text(json.dumps({"x86_64": {
            "url": "https://example.invalid/pinned-ort.tgz",
            "sha256": hashlib.sha256(self.payload).hexdigest()}}))
        self.output = self.root / "download/ort.tgz"

    def fetch(self, payload):
        with mock.patch.object(fetch_ort, "MANIFEST", self.manifest), \
             mock.patch.object(fetch_ort.platform, "machine", return_value="x86_64"), \
             mock.patch.object(fetch_ort.urllib.request, "urlopen", return_value=io.BytesIO(payload)) as get:
            result = fetch_ort.fetch(self.output)
            get.assert_called_once_with("https://example.invalid/pinned-ort.tgz")
            return result

    def test_downloads_manifest_url_and_verifies_the_actual_bytes(self):
        self.assertEqual(self.fetch(self.payload), self.output.resolve())
        self.assertEqual(self.output.read_bytes(), self.payload)

    def test_wrong_sha_fails_and_retains_the_failed_download(self):
        with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
            self.fetch(b"corrupt")
        self.assertEqual(self.output.read_bytes(), b"corrupt")

    def test_existing_output_is_preserved_without_network(self):
        self.output.parent.mkdir()
        self.output.write_bytes(b"keep")
        with mock.patch.object(fetch_ort, "MANIFEST", self.manifest), \
             mock.patch.object(fetch_ort.platform, "machine", return_value="x86_64"), \
             mock.patch.object(fetch_ort.urllib.request, "urlopen") as get:
            with self.assertRaises(FileExistsError):
                fetch_ort.fetch(self.output)
            get.assert_not_called()
        self.assertEqual(self.output.read_bytes(), b"keep")

    def test_explicit_release_mirror_verifies_same_bytes_without_url_fallback(self):
        manifest = json.loads(self.manifest.read_text())
        manifest['mirror_release'] = {'repository': 'example/sdk', 'tag': 'fixed-ort'}
        manifest['x86_64']['directory'] = 'fixed-ort-x64'
        self.manifest.write_text(json.dumps(manifest))
        for payload in (self.payload, b'corrupt'):
            with self.subTest(valid=payload == self.payload):
                def download(argv, *, check):
                    self.assertTrue(check)
                    self.assertEqual(argv, ['gh', 'release', 'download', 'fixed-ort',
                        '--repo', 'example/sdk', '--pattern', 'fixed-ort-x64.tgz',
                        '--output', str(self.output)])
                    self.output.write_bytes(payload)
                with mock.patch.object(fetch_ort, 'MANIFEST', self.manifest), \
                     mock.patch.object(fetch_ort.platform, 'machine', return_value='x86_64'), \
                     mock.patch.object(fetch_ort.subprocess, 'run', side_effect=download) as get, \
                     mock.patch.object(fetch_ort.urllib.request, 'urlopen') as upstream:
                    if payload == self.payload:
                        self.assertEqual(fetch_ort.fetch(self.output), self.output.resolve())
                    else:
                        with self.assertRaisesRegex(ValueError, 'SHA-256 mismatch'):
                            fetch_ort.fetch(self.output)
                    get.assert_called_once()
                    upstream.assert_not_called()
                self.assertEqual(self.output.read_bytes(), payload)
                self.output.unlink()

    def test_release_failure_does_not_fallback_or_touch_existing_output(self):
        manifest = json.loads(self.manifest.read_text())
        manifest['mirror_release'] = {'repository': 'example/sdk', 'tag': 'fixed-ort'}
        manifest['x86_64']['directory'] = 'fixed-ort-x64'
        self.manifest.write_text(json.dumps(manifest))
        with mock.patch.object(fetch_ort, 'MANIFEST', self.manifest), \
             mock.patch.object(fetch_ort.platform, 'machine', return_value='x86_64'), \
             mock.patch.object(fetch_ort.subprocess, 'run',
                               side_effect=subprocess.CalledProcessError(1, ['gh'])) as get, \
             mock.patch.object(fetch_ort.urllib.request, 'urlopen') as upstream:
            with self.assertRaises(subprocess.CalledProcessError):
                fetch_ort.fetch(self.output)
            upstream.assert_not_called()
            self.assertFalse(self.output.exists())
            self.output.write_bytes(b'keep')
            with self.assertRaises(FileExistsError):
                fetch_ort.fetch(self.output)
            get.assert_called_once()
            self.assertEqual(self.output.read_bytes(), b'keep')
