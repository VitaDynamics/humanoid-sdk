"""Offline group contract boundaries; no Node, transport or customer code."""
import re
import unittest

try:
    from .group_contract import GROUP_PREFIX, MAX_GROUP_LEN, new_group, validate_group
except ImportError:
    from group_contract import GROUP_PREFIX, MAX_GROUP_LEN, new_group, validate_group


class GroupContractTest(unittest.TestCase):
    def test_aorta_32_byte_boundary_is_accepted(self):
        group = GROUP_PREFIX + "a" * (32 - len(GROUP_PREFIX))
        self.assertEqual(MAX_GROUP_LEN, 32)
        self.assertEqual(validate_group(group), group)

    def test_33_bytes_are_rejected_with_actual_and_maximum(self):
        with self.assertRaisesRegex(ValueError, r"max length 32 .*got 33"):
            validate_group(GROUP_PREFIX + "a" * (33 - len(GROUP_PREFIX)))

    def test_old_default_is_rejected(self):
        with self.assertRaisesRegex(ValueError, r"max length 32 .*got 45"):
            validate_group(GROUP_PREFIX + "a" * 32)

    def test_default_groups_fit_and_keep_random_identity(self):
        groups = {new_group() for _ in range(1000)}
        self.assertEqual(len(groups), 1000)
        for group in groups:
            self.assertLessEqual(len(group.encode("utf-8")), MAX_GROUP_LEN)
            self.assertTrue(group.startswith(GROUP_PREFIX))
            self.assertIsNotNone(re.fullmatch(r"[A-Za-z0-9_-]+", group))
            self.assertEqual(validate_group(group), group)

    def test_length_counts_utf8_bytes(self):
        with self.assertRaisesRegex(ValueError, r"max length 32 .*got 33"):
            validate_group(GROUP_PREFIX + "a" * 18 + "é")


if __name__ == "__main__":
    unittest.main()
