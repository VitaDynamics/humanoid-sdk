"""Byte-transfer gate tests only: these bytes are NOT robot FlatBuffers."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

_SPEC = importlib.util.spec_from_file_location(
    'verify_c2i_inputs', Path(__file__).resolve().parents[1] / 'tools/verify_c2i_inputs.py')
checker = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(checker)


class C2iInputContractTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.root = self.base / 'capture'
        self.root.mkdir()
        self.manifest = self.base / 'reviewed.json'
        self.compatibility = self.base / 'compatibility.json'
        self.producer = dict(repository='VitaDynamics/vita-robot', target=checker.TARGET,
                             commit='1' * 40, tree='2' * 40,
                             elf_sha256='3' * 64, invocation_sha256='4' * 64)
        self.schemas = {name: hashlib.sha256(name.encode()).hexdigest() for name in checker.SCHEMAS}
        self.expected = dict(format_version=1, producer=self.producer,
                             schema_sha256=self.schemas, files={})
        for name in checker.REQUIRED:
            path = self.root / name
            path.parent.mkdir(exist_ok=True)
            path.write_bytes(b'SYNTHETIC BYTE-GATE TEST ONLY: ' + name.encode())
            self.expected['files'][name] = self.metadata(path)
        self.write_capture(dict(producer=self.producer, schema_sha256=self.schemas))
        self.compatibility.write_text(json.dumps(dict(schema_sha256=self.schemas)))
        self.save()

    @staticmethod
    def metadata(path):
        return dict(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    def write_capture(self, value):
        path = self.root / 'capture.json'
        path.write_text(json.dumps(value))
        self.expected['capture'] = self.metadata(path)

    def save(self):
        self.manifest.write_text(json.dumps(self.expected))

    def verify(self):
        return checker.verify(self.root, self.manifest, self.compatibility)

    def test_complete_reviewed_bytes_are_accepted_without_reencoding(self):
        original = {p: (self.root / p).read_bytes() for p in checker.REQUIRED}
        self.assertEqual(self.verify(), self.root.resolve())
        self.assertEqual({p: (self.root / p).read_bytes() for p in checker.REQUIRED}, original)

    def test_missing_manifest_has_no_empty_pin_fallback(self):
        self.manifest.unlink()
        with self.assertRaisesRegex(ValueError, 'reviewed C2I manifest missing'):
            self.verify()

    def test_missing_or_extra_file_is_not_an_optional_input(self):
        path = self.root / 'static/right_state.fb'
        value = path.read_bytes()
        path.unlink()
        with self.assertRaisesRegex(ValueError, 'exactly 25'):
            self.verify()
        path.write_bytes(value)
        (self.root / 'static/unreviewed.fb').write_bytes(value)
        with self.assertRaisesRegex(ValueError, 'exactly 25'):
            self.verify()

    def test_same_length_corruption_and_wrong_size_fail(self):
        path = self.root / 'brainco/touch.fb'
        original = path.read_bytes()
        path.write_bytes(b'X' + original[1:])
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            self.verify()
        path.write_bytes(original + b'X')
        with self.assertRaisesRegex(ValueError, 'byte count'):
            self.verify()

    def test_symlink_and_unexpected_directory_are_rejected(self):
        path = self.root / 'linker/info.fb'
        original = path.read_bytes()
        outside = self.base / 'outside.fb'
        outside.write_bytes(original)
        path.unlink()
        path.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.verify()
        path.unlink()
        path.write_bytes(original)
        (self.root / 'unreviewed').mkdir()
        with self.assertRaisesRegex(ValueError, 'unexpected C2I directory'):
            self.verify()

    def test_schema_is_checked_against_independent_sdk_pin(self):
        self.expected['schema_sha256'] = dict(self.schemas, end_effector_state='0' * 64)
        self.save()
        with self.assertRaisesRegex(ValueError, 'pinned SDK bindings'):
            self.verify()

    def test_hash_valid_receipt_cannot_change_producer_identity(self):
        self.write_capture(dict(producer=dict(self.producer, commit='9' * 40),
                                schema_sha256=self.schemas))
        self.save()
        with self.assertRaisesRegex(ValueError, 'captured producer/schema identity'):
            self.verify()

    def test_unknown_producer_and_unfixed_identities_fail(self):
        for change in (dict(repository='other/repo'), dict(target='//other:test'),
                       dict(commit=None), dict(tree='moving-main'), dict(elf_sha256=None)):
            with self.subTest(change=change):
                self.expected['producer'] = dict(self.producer, **change)
                self.save()
                with self.assertRaises(ValueError):
                    self.verify()

    def test_duplicate_keys_and_unfixed_byte_metadata_fail(self):
        self.manifest.write_text('{"format_version":1,"format_version":1}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            self.verify()
        self.expected['files']['static/left_info.fb']['sha256'] = None
        self.save()
        with self.assertRaisesRegex(ValueError, 'fixed hexadecimal identity'):
            self.verify()
