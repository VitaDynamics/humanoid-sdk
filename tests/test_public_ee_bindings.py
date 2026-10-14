"""Real public wheel checks; generated messages only, no robot connection."""
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import unittest

from locomotion_aorta._hand_codec import load_bindings

ROOT = Path(__file__).resolve().parents[1]


class PublicEeBindingTest(unittest.TestCase):
    def test_distribution_and_all_pinned_bfbs_match(self):
        manifest = json.loads((ROOT / 'compatibility.json').read_text())
        self.assertEqual(importlib.metadata.version('vbot-humanoid-msgs'),
                         manifest['humanoid_msgs_version'])
        with self.assertRaises(importlib.metadata.PackageNotFoundError):
            importlib.metadata.distribution('vita-msgs')
        for stem, expected in manifest['schema_sha256'].items():
            with self.subTest(stem=stem):
                meta = importlib.import_module(stem + '_schema_meta')
                self.assertEqual(hashlib.sha256(meta.SCHEMA_BFBS).hexdigest(), expected)

    def test_hand_topics_and_both_service_directions_are_generated(self):
        bindings = load_bindings()
        for name in ('HandCmd', 'HandState', 'HandTouch', 'HandInfo',
                     'HandGetInfoRequest', 'HandGetInfoResponse',
                     'HandSetTouchRequest', 'HandSetTouchResponse'):
            with self.subTest(name=name):
                self.assertTrue(callable(getattr(getattr(bindings, name), name).GetRootAs))
        for name in ('HandCmdTrace', 'GripperCmd', 'GripperState', 'GripperInfo'):
            module = importlib.import_module('lowlevel.' + name)
            self.assertTrue(callable(getattr(module, name).GetRootAs))
        for name in ('GripperGetInfoRequest', 'GripperGetInfoResponse'):
            module = importlib.import_module('aorta.services.humanoid_locomotion.' + name)
            self.assertTrue(callable(getattr(module, name).GetRootAs))
