#!/usr/bin/env python3
"""Linker L6 POS-only target entry; no velocity or trajectory guarantee."""
from pathlib import Path
from runpy import run_path

from locomotion_aorta.end_effector_types import DeviceModel


if __name__ == '__main__':
    run_path(str(Path(__file__).with_name('end_effector_external.py')))['main'](
        _expected_device_model=DeviceModel.LINKER_L6)
