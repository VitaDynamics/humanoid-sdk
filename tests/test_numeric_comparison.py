"""Equality policy authorized after the preserved RES-7 signed-zero failure."""
import importlib.util
from pathlib import Path
import struct
import unittest

path=Path(__file__).parent/'fixtures/teleop/compare_robot_oracle.py'
spec=importlib.util.spec_from_file_location('oracle_comparison',path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class NumericEquality(unittest.TestCase):
    def test_ieee_signed_zeros_are_numerically_equal(self):
        self.assertTrue(module.equal_float32(+0.0,-0.0))

    def test_one_float32_ulp_is_rejected_without_tolerance(self):
        next_float=struct.unpack('<f',struct.pack('<I',0x3f800001))[0]
        self.assertFalse(module.equal_float32(1.,next_float))

    def test_nonfinite_values_cannot_pass(self):
        self.assertFalse(module.equal_float32(float('nan'),float('nan')))
        self.assertFalse(module.equal_float32(float('inf'),float('inf')))


if __name__=='__main__':unittest.main()
