#!/usr/bin/env python3
"""Linker L6 POS-only target entry; no velocity or trajectory guarantee."""
import importlib.util
from pathlib import Path

from locomotion_aorta.end_effector_types import DeviceModel


if __name__ == '__main__':
    spec = importlib.util.spec_from_file_location('end_effector_external',
                                                  Path(__file__).with_name('end_effector_external.py'))
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    entry.main(_expected_device_model=DeviceModel.LINKER_L6)
