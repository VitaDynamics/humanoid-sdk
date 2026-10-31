#!/usr/bin/env python3
"""AGX total-opening entry; read-only unless authorized motion is confirmed."""
from pathlib import Path
from runpy import run_path

from locomotion_aorta.end_effector_types import DeviceModel


if __name__ == '__main__':
    run_path(str(Path(__file__).with_name('end_effector_external.py')))['main'](
        _expected_device_model=DeviceModel.AGX)
