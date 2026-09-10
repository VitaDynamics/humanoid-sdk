#!/usr/bin/env python3

import argparse
import math
import time
from collections.abc import Callable, Sequence

from locomotion_aorta import (
    ControlStatus,
    ExternalController,
    ExternalPhase,
    HumanoidLowState,
    LowCmd,
    MotorCommand,
)


COMMAND_SLOTS = 42
PERIOD_S = 0.02
MAX_PEAK_SPEED_RAD_S = 0.10
MIN_SEGMENT_DURATION_S = 2.0
DWELL_S = 1.0
ENDPOINT_SETTLE_S = 0.5
ZERO_ENDPOINT_POSITION_TOLERANCE_RAD = 0.08
RETURN_ENDPOINT_POSITION_TOLERANCE_RAD = 0.05
ELBOW_RETURN_OFFSET_RAD = math.radians(10.0)
STATIONARY_DQ_RAD_S = 0.05
LOWSTATE_WINDOW_S = 0.2
EXIT_MARGIN_S = 5.0
REQUESTED_LEASE_S = 90.0

# A-sample FIXED_STAND baseline with shoulder pitch/elbow kp=80 for tuning.
DEMO_KP = (
    100.0, 100.0, 60.0, 150.0, 80.0, 40.0,
    100.0, 100.0, 60.0, 150.0, 80.0, 40.0,
    80.0, 80.0, 80.0, 40.0, 40.0, 80.0,
    80.0, 40.0, 40.0, 80.0, 60.0, 60.0, 80.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    60.0, 60.0, 80.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    20.0, 20.0,
)
FIXED_STAND_KD = (
    3.0, 3.0, 2.0, 4.0, 2.0, 1.0,
    3.0, 3.0, 2.0, 4.0, 2.0, 1.0,
    2.0, 2.0, 2.0, 1.0, 1.0, 1.0,
    2.0, 1.0, 1.0, 1.0, 3.0, 3.0, 4.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    3.0, 3.0, 4.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 1.0,
)


def run(
    controller: ExternalController,
    *,
    timeout: float,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    emit: Callable[[str], None] = print,
) -> None:
    start_state = _wait_for_stationary_lowstate(
        controller,
        timeout=timeout,
        sleep=sleep,
        monotonic=monotonic,
    )
    start_q = tuple(
        motor.q for motor in start_state.motor_states[:COMMAND_SLOTS]
    )
    return_q = list(start_q)
    # Offset only the elbows toward zero to avoid the observed hip interference.
    for slot in (17, 21):
        return_q[slot] = math.copysign(
            max(abs(start_q[slot]) - ELBOW_RETURN_OFFSET_RAD, 0.0),
            start_q[slot],
        )
    segment_steps = _segment_steps(start_q)
    motion_duration_s = 2 * segment_steps * PERIOD_S + 2 * DWELL_S
    required_lease_s = motion_duration_s + EXIT_MARGIN_S
    if required_lease_s > REQUESTED_LEASE_S:
        raise ValueError(
            f"trajectory does not fit the {REQUESTED_LEASE_S:g}-second lease: "
            f"requires {required_lease_s:.2f}s including exit margin"
        )
    emit(
        f"preflight passed: {COMMAND_SLOTS} slots, "
        f"trajectory={motion_duration_s:.2f}s, "
        f"required lease={required_lease_s:.2f}s"
    )
    emit(
        "elbow return targets (10 deg toward zero, clamped at zero): "
        f"slot 17 {start_q[17]:.4f} -> {return_q[17]:.4f} rad, "
        f"slot 21 {start_q[21]:.4f} -> {return_q[21]:.4f} rad"
    )

    entered = False
    try:
        entered_status = controller.enter_external(
            timeout, lease_duration_s=REQUESTED_LEASE_S
        )
        entered = True
        remaining_lease_s = _remaining_lease_seconds(entered_status)
        if remaining_lease_s < required_lease_s:
            raise RuntimeError(
                "actual remaining lease is too short: "
                f"{remaining_lease_s:.2f}s available, "
                f"{required_lease_s:.2f}s required"
            )

        generation = entered_status.generation
        reject_count = entered_status.reject_count
        client_id = entered_status.client_id
        lease_id = entered_status.lease_id
        last_state_id = start_state.state_id
        last_state_change = monotonic()
        next_publish = monotonic()

        def publish_frame(q: Sequence[float]) -> HumanoidLowState:
            nonlocal last_state_id, last_state_change, next_publish
            delay = next_publish - monotonic()
            if delay > 0:
                sleep(delay)
            status = controller.status
            if (
                status is None
                or status.current_action != "EXTERNAL"
                or status.phase != ExternalPhase.RUNNING
                or status.generation != generation
                or status.client_id != client_id
                or status.lease_id != lease_id
            ):
                raise RuntimeError(
                    "EXTERNAL ownership is no longer the entered RUNNING session"
                )
            if status.reject_count != reject_count:
                raise RuntimeError(
                    "robot reject_count changed during the trajectory"
                )

            state = controller.wait_lowstate(timeout)
            _validate_lowstate(state, require_stationary=False)
            now = monotonic()
            if state.state_id != last_state_id:
                last_state_id = state.state_id
                last_state_change = now
            elif now - last_state_change >= LOWSTATE_WINDOW_S:
                raise RuntimeError("lowstate became stale during the trajectory")

            controller.publish(_position_command(q))
            next_publish = max(next_publish + PERIOD_S, monotonic() + PERIOD_S)
            return state

        zero_q = (0.0,) * COMMAND_SLOTS
        publish_frame(start_q)
        _publish_segment(publish_frame, start_q, zero_q, segment_steps)
        _publish_dwell(
            publish_frame,
            zero_q,
            monotonic=monotonic,
            label="zero",
            position_tolerance_rad=ZERO_ENDPOINT_POSITION_TOLERANCE_RAD,
        )
        emit("zero endpoint settled")
        _publish_segment(publish_frame, zero_q, return_q, segment_steps)
        _publish_dwell(
            publish_frame,
            return_q,
            monotonic=monotonic,
            label="return",
            position_tolerance_rad=RETURN_ENDPOINT_POSITION_TOLERANCE_RAD,
        )
        emit("return endpoint settled")
    finally:
        if entered:
            exit_status = controller.exit_external(timeout)
            if exit_status.current_action != "PASSIVE":
                raise RuntimeError(
                    "CANCEL cleared EXTERNAL but expected PASSIVE, got "
                    f"{exit_status.current_action or '<empty>'}"
                )
            emit("CANCEL confirmed PASSIVE")


def _wait_for_stationary_lowstate(
    controller: ExternalController,
    *,
    timeout: float,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
) -> HumanoidLowState:
    deadline = monotonic() + timeout
    stationary_since: float | None = None
    last_new_time = monotonic()
    last_state_id: int | None = None
    latest: HumanoidLowState | None = None

    while monotonic() < deadline:
        remaining = deadline - monotonic()
        latest = controller.wait_lowstate(max(remaining, 0.001))
        _validate_lowstate(latest, require_stationary=False)
        now = monotonic()
        if latest.state_id != last_state_id:
            last_state_id = latest.state_id
            last_new_time = now
            if any(
                abs(motor.dq) > STATIONARY_DQ_RAD_S
                for motor in latest.motor_states[:COMMAND_SLOTS]
            ):
                stationary_since = None
            elif stationary_since is None:
                stationary_since = now
            elif now - stationary_since >= LOWSTATE_WINDOW_S:
                return latest
        elif now - last_new_time >= LOWSTATE_WINDOW_S:
            raise RuntimeError("lowstate is stale during the preflight window")
        sleep(min(PERIOD_S, max(0.0, deadline - now)))

    raise RuntimeError("timed out waiting for 200 ms of fresh stationary lowstate")


def _validate_lowstate(
    state: HumanoidLowState, *, require_stationary: bool
) -> None:
    if not state.attitude_valid:
        raise RuntimeError("lowstate attitude is invalid")
    if len(state.motor_states) < COMMAND_SLOTS:
        raise RuntimeError(
            f"lowstate has {len(state.motor_states)} motor slots; "
            f"{COMMAND_SLOTS} required"
        )
    for index, motor in enumerate(state.motor_states[:COMMAND_SLOTS]):
        if not math.isfinite(motor.q) or not math.isfinite(motor.dq):
            raise RuntimeError(f"lowstate motor[{index}] q/dq is not finite")
        if require_stationary and abs(motor.dq) > STATIONARY_DQ_RAD_S:
            raise RuntimeError(
                f"lowstate motor[{index}] is moving at {motor.dq:.3f} rad/s"
            )


def _segment_steps(start_q: Sequence[float]) -> int:
    displacement = max(abs(q) for q in start_q)
    duration_s = max(
        MIN_SEGMENT_DURATION_S,
        1.5 * displacement / MAX_PEAK_SPEED_RAD_S,
    )
    return math.ceil(duration_s / PERIOD_S)


def _remaining_lease_seconds(status: ControlStatus) -> float:
    deadline_ns = status.lease_deadline_monotonic_ns
    stamp_ns = status.stamp_monotonic_ns
    if deadline_ns <= stamp_ns or stamp_ns <= 0:
        raise RuntimeError("RUNNING status did not provide a usable lease deadline")
    return (deadline_ns - stamp_ns) / 1_000_000_000


def _position_command(q: Sequence[float]) -> LowCmd:
    if len(q) != COMMAND_SLOTS:
        raise ValueError(f"position command must contain {COMMAND_SLOTS} slots")
    commands = tuple(
        MotorCommand(mode=10, q=position, kp=kp, kd=kd)
        for position, kp, kd in zip(q, DEMO_KP, FIXED_STAND_KD)
    )
    return LowCmd(cmd_id=1, motor_cmds=commands)


def _publish_segment(
    publish: Callable[[Sequence[float]], HumanoidLowState],
    start: Sequence[float],
    target: Sequence[float],
    steps: int,
) -> None:
    for step in range(1, steps + 1):
        u = step / steps
        alpha = u * u * (3.0 - 2.0 * u)
        publish(
            tuple(
                start_q + (target_q - start_q) * alpha
                for start_q, target_q in zip(start, target)
            )
        )


def _publish_dwell(
    publish: Callable[[Sequence[float]], HumanoidLowState],
    target: Sequence[float],
    *,
    monotonic: Callable[[], float],
    label: str,
    position_tolerance_rad: float,
) -> None:
    settled_since: float | None = None
    settled = False
    for _ in range(math.ceil(DWELL_S / PERIOD_S)):
        state = publish(target)
        now = monotonic()
        at_endpoint = all(
            abs(motor.q - target_q) <= position_tolerance_rad
            and abs(motor.dq) <= STATIONARY_DQ_RAD_S
            for motor, target_q in zip(
                state.motor_states[:COMMAND_SLOTS], target
            )
        )
        if at_endpoint:
            if settled_since is None:
                settled_since = now
            elif now - settled_since >= ENDPOINT_SETTLE_S:
                settled = True
        else:
            settled_since = None
            settled = False
    if not settled:
        raise RuntimeError(
            f"{label} endpoint did not settle within position/velocity limits"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Hoist-only 42-slot EXTERNAL demo: current pose to zero, "
            "then return with elbow clearance offset"
        )
    )
    parser.add_argument("--group", default="default")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="required acknowledgement that this command may move the robot",
    )
    args = parser.parse_args()
    if not args.confirm_motion:
        parser.error("--confirm-motion is required")

    with ExternalController(group=args.group) as controller:
        run(controller, timeout=args.timeout)


if __name__ == "__main__":
    main()
