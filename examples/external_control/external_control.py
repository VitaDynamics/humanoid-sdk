#!/usr/bin/env python3

import argparse
import json
import math
import time
from collections.abc import Callable, Sequence
from pathlib import Path

from locomotion_aorta import (
    PullController,
    ExternalPhase,
    BodyStateArrays,
    BodyCmd,
    BodyJointCmd,
    BodyCmdAction,
    JointError,
    BODY_SLOTS,
    ServiceStatus,
)


COMMAND_SLOTS = BODY_SLOTS
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

# A-sample FIXED_STAND baseline with shoulder pitch/elbow kp=80 for tuning.
DEMO_KP = (100.0, 100.0, 60.0, 150.0, 80.0, 40.0, 100.0, 100.0, 60.0, 150.0, 80.0, 40.0, 80.0, 80.0, 0.0, 80.0, 40.0, 40.0, 80.0, 60.0, 60.0, 80.0, 80.0, 40.0, 40.0, 80.0, 60.0, 60.0, 80.0, 20.0, 20.0)
FIXED_STAND_KD = (3.0, 3.0, 2.0, 4.0, 2.0, 1.0, 3.0, 3.0, 2.0, 4.0, 2.0, 1.0, 2.0, 2.0, 0.0, 2.0, 1.0, 1.0, 1.0, 3.0, 3.0, 4.0, 2.0, 1.0, 1.0, 1.0, 3.0, 3.0, 4.0, 1.0, 1.0)


def _matches_external_identity(status, generation, client_id, control_id):
    return not (status.generation != generation or status.client_id != client_id
                or status.control_id != control_id)


def _external_status_problem(status, phases, entered):
    if status is None or status.current_action != 'EXTERNAL' or status.phase not in phases:
        return 'owner'
    if not _matches_external_identity(status, entered.generation, entered.client_id, entered.control_id):
        return 'owner'
    if status.reject_count != entered.reject_count:
        return 'reject'
    return None


def _body_sample_progress(state_id, last_id, last_change, now, window):
    if state_id != last_id:
        return state_id, now, False
    return last_id, last_change, now - last_change >= window


def run(
    controller: PullController,
    *,
    timeout: float,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    emit: Callable[[str], None] = print,
) -> None:
    info = require_body_info(controller, timeout, emit_info=emit)
    target_slots = _body_target_slots(info, require_zero=True)
    target_names = tuple(info.joint_names[i] for i in target_slots)
    start_state = _wait_for_stationary_body_state(
        controller,
        body_info=info,
        timeout=timeout,
        sleep=sleep,
        monotonic=monotonic,
        target_slots=target_slots,
    )
    positions = start_state.q
    start_q = tuple(float(q) if i in target_slots else 0.0 for i, q in enumerate(positions))
    return_q = list(start_q)
    # Offset only the elbows toward zero to avoid the observed hip interference.
    for slot in (18, 25):
        return_q[slot] = math.copysign(
            max(abs(start_q[slot]) - ELBOW_RETURN_OFFSET_RAD, 0.0),
            start_q[slot],
        )
    segment_steps = _segment_steps(start_q)
    motion_duration_s = 2 * segment_steps * PERIOD_S + 2 * DWELL_S
    emit(
        f"preflight passed: {COMMAND_SLOTS} slots, "
        f"trajectory={motion_duration_s:.2f}s"
    )
    emit(
        "elbow return targets (10 deg toward zero, clamped at zero): "
        f"slot 18 {start_q[18]:.4f} -> {return_q[18]:.4f} rad, "
        f"slot 25 {start_q[25]:.4f} -> {return_q[25]:.4f} rad"
    )

    entered = False
    try:
        entered_status = controller.enter_external(timeout)
        entered = True

        last_state_id = start_state.state_id
        last_state_change = monotonic()
        next_publish = monotonic()

        def publish_frame(q: Sequence[float]) -> BodyStateArrays:
            nonlocal last_state_id, last_state_change, next_publish
            delay = next_publish - monotonic()
            if delay > 0:
                sleep(delay)
            status = controller.status
            problem = _external_status_problem(status, (ExternalPhase.RUNNING,), entered_status)
            if problem == 'owner':
                raise RuntimeError(
                    "EXTERNAL ownership is no longer the entered RUNNING session"
                )
            if problem == 'reject':
                raise RuntimeError(
                    "robot reject_count changed during the trajectory"
                )

            state = controller.wait_state(timeout)
            _validate_body_state(state, body_info=info, require_stationary=False, target_slots=target_slots)
            now = monotonic()
            last_state_id, last_state_change, stale = _body_sample_progress(
                state.state_id, last_state_id, last_state_change, now, LOWSTATE_WINDOW_S)
            if stale:
                raise RuntimeError("body state became stale during the trajectory")

            controller.publish(_position_command(q, state), target_names=target_names, required_fields=0x19)
            next_publish = max(next_publish + PERIOD_S, monotonic() + PERIOD_S)
            return state

        zero_q = (0.0,) * COMMAND_SLOTS
        publish_frame(start_q)
        _publish_segment(publish_frame, start_q, zero_q, segment_steps)
        _publish_dwell(
            publish_frame,
            zero_q,
            body_info=info,
            monotonic=monotonic,
            label="zero",
            position_tolerance_rad=ZERO_ENDPOINT_POSITION_TOLERANCE_RAD,
            target_slots=target_slots,
        )
        emit("zero endpoint settled")
        _publish_segment(publish_frame, zero_q, return_q, segment_steps)
        _publish_dwell(
            publish_frame,
            return_q,
            body_info=info,
            monotonic=monotonic,
            label="return",
            position_tolerance_rad=RETURN_ENDPOINT_POSITION_TOLERANCE_RAD,
            target_slots=target_slots,
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


def require_body_info(controller: PullController, timeout: float, *, emit_info=None):
    response = controller.get_robot_info(timeout)
    if response.status not in (ServiceStatus.SUCCESS, ServiceStatus.PARTIAL_SUCCESS):
        raise RuntimeError(f"get_robot_info failed: status={response.status} {response.error_message}")
    info = response.info
    body = None if info is None else info.body
    if (body is None or body.slot_count != COMMAND_SLOTS
            or any(len(v) != COMMAND_SLOTS for v in (body.joint_names, body.joints, body.motor_identities))):
        raise RuntimeError("get_robot_info did not provide the public31 body roster")
    if emit_info is not None:
        from locomotion_aorta._policy_state import format_robot_info
        emit_info(format_robot_info(response))
    return body


def _body_target_slots(body_info, *, require_zero=False):
    """Validate the existing FIXED_STAND 30-joint action profile, not slot presence."""
    from locomotion_aorta._policy_state import body_joint_order

    path = Path(__file__).resolve().parents[1] / 'fixed_stand_pose/fixed_stand_profile.json'
    names = tuple(body_info.joint_names)
    targets = frozenset(json.loads(path.read_text())['joint_gains'])
    if (body_info.slot_count != COMMAND_SLOTS or len(body_info.joints) != COMMAND_SLOTS
            or len(targets) != 30 or not targets.issubset(names)
            or any(cap.body_index != i for i, cap in enumerate(body_info.joints))):
        raise ValueError('the full 30-joint Body action profile is required')
    body_joint_order(body_info, required_names=targets)
    slots = tuple(i for i, name in enumerate(names) if name in targets)
    if require_zero:
        for i in slots:
            cap = body_info.joints[i]
            if not cap.q_min <= 0.0 <= cap.q_max:
                raise ValueError(f'{names[i]}: zero outside position limit')
    return slots


def _wait_for_stationary_body_state(
    controller: PullController,
    *,
    body_info,
    timeout: float,
    sleep: Callable[[float], None],
    monotonic: Callable[[], float],
    target_slots=None,
) -> BodyStateArrays:
    if target_slots is None:
        target_slots = _body_target_slots(body_info)
    if not target_slots:
        raise ValueError('Body action has no target slots')
    deadline = monotonic() + timeout
    stationary_since: float | None = None
    last_new_time = monotonic()
    last_state_id: int | None = None
    latest: BodyStateArrays | None = None

    while monotonic() < deadline:
        remaining = deadline - monotonic()
        latest = controller.wait_state(max(remaining, 0.001))
        _validate_body_state(latest, body_info=body_info, require_stationary=False, target_slots=target_slots)
        now = monotonic()
        if latest.state_id != last_state_id:
            last_state_id = latest.state_id
            last_new_time = now
            if any(
                abs(dq) > STATIONARY_DQ_RAD_S
                for i, dq in enumerate(latest.dq) if i in target_slots
            ):
                stationary_since = None
            elif stationary_since is None:
                stationary_since = now
            elif now - stationary_since >= LOWSTATE_WINDOW_S:
                return latest
        elif now - last_new_time >= LOWSTATE_WINDOW_S:
            raise RuntimeError("body state is stale during the preflight window")
        sleep(min(PERIOD_S, max(0.0, deadline - now)))

    raise RuntimeError("timed out waiting for 200 ms of fresh stationary body state")


def _validate_body_state(
    state: BodyStateArrays, *, body_info, require_stationary: bool, target_slots=None
) -> None:
    if target_slots is None:
        target_slots = _body_target_slots(body_info)
    if not target_slots:
        raise ValueError('Body action has no target slots')
    if not state.attitude_valid:
        raise RuntimeError("body state attitude is invalid")
    q, dq, errors = state.q, state.dq, state.error_code
    if any(len(column) != COMMAND_SLOTS for column in (q, dq, errors)):
        raise RuntimeError(f"body state requires {COMMAND_SLOTS} motor slots")
    for index in target_slots:
        position, velocity, error = q[index], dq[index], errors[index]
        if error != JointError.NONE:
            raise RuntimeError(f"body state motor[{index}] error_code={error}")
        if not math.isfinite(position) or not math.isfinite(velocity):
            raise RuntimeError(f"body state motor[{index}] q/dq is not finite")
        cap = body_info.joints[index]
        if not cap.q_min <= position <= cap.q_max:
            raise RuntimeError(f"body state motor[{index}] q is outside BodyInfo limit")
        if require_stationary and abs(velocity) > STATIONARY_DQ_RAD_S:
            raise RuntimeError(f"body state motor[{index}] is moving at {velocity:.3f} rad/s")


def _segment_steps(start_q: Sequence[float]) -> int:
    displacement = max(abs(q) for q in start_q)
    duration_s = max(
        MIN_SEGMENT_DURATION_S,
        1.5 * displacement / MAX_PEAK_SPEED_RAD_S,
    )
    return math.ceil(duration_s / PERIOD_S)


def _position_command(q: Sequence[float], state: BodyStateArrays | None = None) -> BodyCmd:
    if len(q) != COMMAND_SLOTS:
        raise ValueError(f"position command must contain {COMMAND_SLOTS} slots")
    commands = tuple(
        BodyJointCmd(q=position, kp=kp, kd=kd)
        for position, kp, kd in zip(q, DEMO_KP, FIXED_STAND_KD)
    )
    return BodyCmd(joints=commands, action=BodyCmdAction.EXTERNAL,
                  input_state_id=0 if state is None else state.state_id,
                  input_state_ts_real_us=0 if state is None else state.ts_state_real_us)


def _publish_segment(
    publish: Callable[[Sequence[float]], BodyStateArrays],
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
    publish: Callable[[Sequence[float]], BodyStateArrays],
    target: Sequence[float],
    *,
    body_info,
    monotonic: Callable[[], float],
    label: str,
    position_tolerance_rad: float,
    target_slots=None,
) -> None:
    if target_slots is None:
        target_slots = _body_target_slots(body_info)
    if not target_slots:
        raise ValueError('Body action has no target slots')
    settled_since: float | None = None
    settled = False
    for _ in range(math.ceil(DWELL_S / PERIOD_S)):
        state = publish(target)
        now = monotonic()
        at_endpoint = all(
            abs(state.q[i] - target[i]) <= position_tolerance_rad
            and abs(state.dq[i]) <= STATIONARY_DQ_RAD_S
            for i in target_slots
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
            "Hoist-only 31-slot EXTERNAL demo: current pose to zero, "
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

    with PullController(group=args.group) as controller:
        run(controller, timeout=args.timeout)


if __name__ == "__main__":
    main()
