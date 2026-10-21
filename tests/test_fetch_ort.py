"""Exercise the real archive supply path without network or compilation."""
import hashlib
import io
import json
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
