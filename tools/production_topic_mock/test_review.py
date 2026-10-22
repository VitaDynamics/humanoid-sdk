"""Functional boundary tests retained after deleting latency-report machinery."""
import unittest
from .run import validate_imu
from . import validate_auxiliary

class FixtureReviewTest(unittest.TestCase):
    def test_imu_does_not_silently_normalize(self):
        with self.assertRaises(ValueError):
            validate_imu([[2, 0, 0, 0], [1, 0, 0, 0]])

    def test_joy_frames_require_increasing_time(self):
        with self.assertRaises(ValueError):
            validate_auxiliary({"joy": [dict(at_ms=1, axes=[], buttons=[])] * 2})
