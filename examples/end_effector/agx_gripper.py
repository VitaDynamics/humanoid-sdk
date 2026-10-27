#!/usr/bin/env python3
"""AGX total-opening entry; read-only unless authorized motion is confirmed."""
from end_effector_external import main
from locomotion_aorta.end_effector_types import DeviceModel


if __name__ == '__main__':
    main(_expected_device_model=DeviceModel.AGX)
