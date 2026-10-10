"""Build-input integrity checks; no download or compiler invocation."""
import hashlib
import io
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent / "latency"))
import build_mock


class LatencyBuildTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.archive = self.directory / "flatbuffers.tar.gz"

    def archive_with(self, name="flatbuffers-25.9.23/CMakeLists.txt"):
        with tarfile.open(self.archive, "w:gz") as archive:
            data = b"pinned source\n"
            entry = tarfile.TarInfo(name)
            entry.size = len(data)
            archive.addfile(entry, io.BytesIO(data))
        return hashlib.sha256(self.archive.read_bytes()).hexdigest()

    def test_reuse_restores_modified_missing_and_extra_files(self):
        with patch.object(build_mock, "FB_SHA", self.archive_with()):
            source = build_mock.extract_source(self.archive, self.directory)
            (source / "CMakeLists.txt").write_text("modified")
            (source / "extra.cmake").write_text("untrusted extra input")
            cache = self.directory / "flatbuffers-build"
            cache.mkdir()
            (cache / "kept.o").write_bytes(b"cached object")
            build_mock.extract_source(self.archive, self.directory)
            self.assertEqual((source / "CMakeLists.txt").read_text(), "pinned source\n")
            self.assertFalse((source / "extra.cmake").exists())
            self.assertEqual((cache / "kept.o").read_bytes(), b"cached object")
            (source / "CMakeLists.txt").unlink()
            build_mock.extract_source(self.archive, self.directory)
            self.assertTrue((source / "CMakeLists.txt").is_file())

    def test_bad_archive_does_not_replace_existing_source(self):
        with patch.object(build_mock, "FB_SHA", self.archive_with()):
            source = build_mock.extract_source(self.archive, self.directory)
            self.archive.write_bytes(b"corrupt")
            with self.assertRaisesRegex(RuntimeError, "checksum"):
                build_mock.extract_source(self.archive, self.directory)
            self.assertEqual((source / "CMakeLists.txt").read_text(), "pinned source\n")

    def test_unexpected_paths_rejected(self):
        for name in ("../escape", "/absolute", "other/file", "flatbuffers-25.9.23/../escape"):
            with self.subTest(name=name), patch.object(build_mock, "FB_SHA", self.archive_with(name)):
                with self.assertRaisesRegex(RuntimeError, "unsafe"):
                    build_mock.extract_source(self.archive, self.directory)

    def test_source_symlink_not_followed(self):
        outside = self.directory / "not-build-inputs"
        outside.mkdir()
        (outside / "keep").write_text("keep")
        (self.directory / "flatbuffers-25.9.23").symlink_to(outside, target_is_directory=True)
        with patch.object(build_mock, "FB_SHA", self.archive_with()):
            with self.assertRaisesRegex(RuntimeError, "symlink"):
                build_mock.extract_source(self.archive, self.directory)
        self.assertEqual((outside / "keep").read_text(), "keep")
