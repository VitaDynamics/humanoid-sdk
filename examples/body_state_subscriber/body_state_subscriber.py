#!/usr/bin/env python3

import argparse
from collections.abc import Callable
from contextlib import closing

from locomotion_aorta import BodyStateSource


def run_once(
    source: BodyStateSource,
    *,
    timeout: float,
    emit: Callable[[str], None] = print,
) -> None:
    state = source.snapshot(timeout).body_state()
    emit(f"state_id={state.state_id} ts_state_real_us={state.ts_state_real_us}")
    for index, motor in enumerate(state.joints):
        emit(f"joint[{index}] error_code={motor.error_code} q={motor.q:.6f} dq={motor.dq:.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Read one public31 body state")
    parser.add_argument("--group", default="default")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    with closing(BodyStateSource(group=args.group)) as source:
        run_once(source, timeout=args.timeout)


if __name__ == "__main__":
    main()
