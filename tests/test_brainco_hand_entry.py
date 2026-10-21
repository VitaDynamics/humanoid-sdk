"""Canonical hand entry rejects obsolete slot specs before opening transport."""
import unittest
import io
from unittest.mock import patch
from examples.brainco_hand import brainco_hand_external

class HandEntryTest(unittest.TestCase):
    def test_old_slot_spec_is_rejected_before_opening_any_node(self):
        with patch.object(brainco_hand_external, 'HandClient') as node, \
             patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit):
                brainco_hand_external.main(['--spec', 'old-reviewed-spec.json'])
            node.assert_not_called()
