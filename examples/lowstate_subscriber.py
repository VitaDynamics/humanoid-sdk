#!/usr/bin/env python3

import argparse
from collections.abc import Callable

from locomotion_aorta import ExternalController


def run_once(
    controller: ExternalController,
    *,
    timeout: float,
    emit: Callable[[str], None] = print,
) -> None:
    state = controller.wait_lowstate(timeout)
    emit(f"state_id={state.state_id} stamp_ns={state.stamp_ns}")
    for index, motor in enumerate(state.motor_states):
        emit(f"joint[{index}] q={motor.q:.6f} dq={motor.dq:.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Read one humanoid lowstate")
    parser.add_argument("--group", default="default")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    with ExternalController(group=args.group) as controller:
        run_once(controller, timeout=args.timeout)


if __name__ == "__main__":
    main()
