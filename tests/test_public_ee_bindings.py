"""Real public wheel checks; generated messages only, no robot connection."""
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import unittest

import locomotion_aorta

from locomotion_aorta._end_effector_codec import load_bindings, load_touch_bindings

ROOT = Path(__file__).resolve().parents[1]


class PublicEeBindingTest(unittest.TestCase):
    def test_t021_struct_layouts_and_message_level_clocks(self):
        from lowlevel.ee.AxisCommand import AxisCommand
        from lowlevel.ee.AxisState import AxisState
        from lowlevel.SampleRef import SampleRef
        from lowlevel.ee.EndEffectorState import EndEffectorState
        self.assertEqual((AxisCommand.SizeOf(), AxisState.SizeOf(), SampleRef.SizeOf()),
                         (28, 28, 32))
        self.assertEqual(AxisCommand.__module__, 'lowlevel.ee.AxisCommand')
        self.assertEqual(AxisState.__module__, 'lowlevel.ee.AxisState')
        for name in ('CommandFields', 'Mode', 'Pos', 'Vel', 'VelLimit', 'Tau', 'Current'):
            self.assertTrue(hasattr(AxisCommand, name))
        for name in ('ValidFields', 'Pos', 'Vel', 'Tau', 'Current', 'TemperatureC', 'ErrorCode'):
            self.assertTrue(hasattr(AxisState, name))
        for name in ('TsAcquiredHwUs', 'TsIngestedUs', 'TsAssembledUs'):
            self.assertTrue(hasattr(EndEffectorState, name))
            self.assertFalse(hasattr(AxisState, name))
        for name in ('Dof', 'TsPollRequestUs', 'TsPollResponseUs', 'CmdDequeuedUs'):
            self.assertFalse(hasattr(EndEffectorState, name))
        # The schema wheel also serves legacy robot consumers. SDK retirement
        # forbids these public SDK exports, not their existence in that wheel.
        for name in ('HandClient', 'GripperClient', 'EndEffectorKind',
                     'HandJointCmd', 'HandJointState', 'FingerTouchMeta', 'HandCmd',
                     'HandState', 'HandTouch', 'HandInfo', 'GripperCmd',
                     'GripperState', 'GripperInfo'):
            with self.subTest(retired=name):
                self.assertFalse(hasattr(locomotion_aorta, name))
                self.assertNotIn(name, locomotion_aorta.__all__)

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

    def test_generic_topics_and_touch_control_are_generated(self):
        bindings = load_bindings()
        for name in ('EndEffectorCmd', 'EndEffectorState', 'EndEffectorTouch', 'EndEffectorInfo'):
            with self.subTest(name=name):
                self.assertTrue(callable(getattr(getattr(bindings, name), name).GetRootAs))
        service = load_touch_bindings()
        for module, name in ((service.request, 'EndEffectorSetTouchRequest'),
                             (service.response, 'EndEffectorSetTouchResponse')):
            self.assertTrue(callable(getattr(module, name).GetRootAs))
        for name in ('hand_cmd_trace',):
            with self.subTest(removed_root=name), self.assertRaises(ImportError):
                importlib.import_module(name + '_schema_meta')
