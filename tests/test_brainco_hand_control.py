"""Regression: the old entry is now the independent-hand example, never slot control."""
import unittest
import io
from unittest.mock import patch
from examples import brainco_hand_control, brainco_hand_external

class LegacyEntryTest(unittest.TestCase):
    def test_legacy_entry_uses_new_main(self):
        self.assertIs(brainco_hand_control.main, brainco_hand_external.main)

    def test_old_slot_spec_is_rejected_before_opening_any_node(self):
        with patch.object(brainco_hand_external, 'HandClient') as node, \
             patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit):
                brainco_hand_control.main(['--spec', 'old-reviewed-spec.json'])
            node.assert_not_called()
