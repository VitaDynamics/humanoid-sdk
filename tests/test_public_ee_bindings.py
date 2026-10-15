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
    def test_t021_struct_layouts_and_message_level_clocks(self):
        from lowlevel.HandJointCmd import HandJointCmd
        from lowlevel.HandJointState import HandJointState
        from lowlevel.FingerTouchMeta import FingerTouchMeta
        from lowlevel.HandState import HandState
        from lowlevel.HandInfo import HandInfo
        self.assertEqual((HandJointCmd.SizeOf(), HandJointState.SizeOf(), FingerTouchMeta.SizeOf()),
                         (8, 28, 32))
        for name in ('TsAcquiredHwUs', 'TsIngestedUs', 'TsAssembledUs'):
            self.assertTrue(hasattr(HandState, name))
            self.assertFalse(hasattr(HandJointState, name))
        for name in ('Dof', 'TsPollRequestUs', 'TsPollResponseUs', 'CmdDequeuedUs'):
            self.assertFalse(hasattr(HandState, name))
        self.assertFalse(hasattr(HandInfo, 'Model'))

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

    def test_hand_topics_and_touch_control_are_generated(self):
        bindings = load_bindings()
        for name in ('HandCmd', 'HandState', 'HandTouch', 'HandInfo',
                     'HandSetTouchRequest', 'HandSetTouchResponse'):
            with self.subTest(name=name):
                self.assertTrue(callable(getattr(getattr(bindings, name), name).GetRootAs))
        for name in ('hand_cmd_trace',):
            with self.subTest(removed_root=name), self.assertRaises(ImportError):
                importlib.import_module(name + '_schema_meta')
