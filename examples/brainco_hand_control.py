#!/usr/bin/env python3
"""Reviewed-baseline Revo2 EXTERNAL test; defaults to offline validation.

No device discovery, mode writes, automatic return trajectory or motion retry.
Whole-body baseline and all limits must come from a separately reviewed spec.
"""

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import time

from locomotion_aorta import ExternalController, ExternalPhase, LowCmd, MotorCommand


COMMAND_SLOTS = 42
STATE_SLOTS = 44
HAND_SLOTS = tuple(range(25, 31)) + tuple(range(34, 40))
PERIOD_S = 0.02
PREFLIGHT_WINDOW_S = 0.2
MOTOR_FIELDS = {"mode", "q", "dq", "tau", "kp", "kd"}


def _number(value, name, *, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    return float(value)


def _reject_constant(value):
    raise ValueError(f"invalid JSON number {value}")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


@dataclass(frozen=True)
class Plan:
    baseline: tuple[MotorCommand, ...]
    targets: tuple[tuple[int, float], ...]
    tolerances: tuple[float, ...]
    move_duration_s: float
    settle_duration_s: float
    max_feedback_gap_s: float
    max_status_gap_s: float
    stationary_dq_rad_s: float
    sha256: str


def load_plan(path: Path) -> Plan:
    data = path.read_bytes()
    spec = json.loads(data, parse_constant=_reject_constant, object_pairs_hook=_unique_object)
    if not isinstance(spec, dict):
        raise ValueError("spec must be an object")
    required = {
        "format_version", "robot_profile", "reviewed_by", "robot_config_sha256",
        "device_preflight_evidence", "stop_method", "baseline", "hand_limits",
        "target_positions_rad", "position_tolerance_rad", "move_duration_s",
        "settle_duration_s", "max_feedback_gap_s", "max_status_gap_s",
        "stationary_dq_rad_s",
    }
    if set(spec) != required:
        raise ValueError(f"spec fields differ: {sorted(set(spec) ^ required)}")
    if type(spec["format_version"]) is not int or spec["format_version"] != 1:
        raise ValueError("format_version must be 1")
    if spec["robot_profile"] != "foot_humanoid_asample_brainco":
        raise ValueError("requires full-body A-sample BrainCo profile")
    for name in ("reviewed_by", "device_preflight_evidence", "stop_method"):
        if (not isinstance(spec[name], str) or not spec[name].strip()
                or spec[name].strip() in ("P0_REQUIRED", "TBD")):
            raise ValueError(f"{name} must identify the actual reviewed evidence")
    config_hash = spec["robot_config_sha256"]
    if (not isinstance(config_hash, str) or len(config_hash) != 64
            or any(char not in "0123456789abcdef" for char in config_hash)):
        raise ValueError("robot_config_sha256 must be a lowercase SHA256")
    rows = spec["baseline"]
    if not isinstance(rows, list) or len(rows) != COMMAND_SLOTS:
        raise ValueError("baseline must explicitly provide all 42 motor commands")
    baseline = []
    for slot, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != MOTOR_FIELDS:
            raise ValueError(f"baseline[{slot}] must provide mode/q/dq/tau/kp/kd")
        if type(row["mode"]) is not int or row["mode"] != 10:
            raise ValueError("reviewed static baseline requires mode=10")
        for field in MOTOR_FIELDS - {"mode"}:
            _number(row[field], f"baseline[{slot}].{field}")
        if row["dq"] != 0 or row["kp"] < 0 or row["kd"] < 0:
            raise ValueError("static baseline requires dq=0 and nonnegative gains")
        if slot in HAND_SLOTS and any(row[field] != 0 for field in ("dq", "tau", "kp", "kd")):
            raise ValueError("BrainCo hand commands support position only")
        baseline.append(MotorCommand(**row))
    tolerance = spec["position_tolerance_rad"]
    if not isinstance(tolerance, list) or len(tolerance) != COMMAND_SLOTS:
        raise ValueError("position_tolerance_rad must have 42 approved values")
    tolerance = tuple(_number(v, "position tolerance", positive=True) for v in tolerance)
    limits = spec["hand_limits"]
    if not isinstance(limits, dict) or set(limits) != {str(slot) for slot in HAND_SLOTS}:
        raise ValueError("hand_limits must cover exactly slots 25..30 and 34..39")
    for slot in HAND_SLOTS:
        limit = limits[str(slot)]
        if not isinstance(limit, dict) or set(limit) != {"lower_rad", "upper_rad", "max_target_rate_rad_s"}:
            raise ValueError(f"invalid hand_limits[{slot}]")
        low = _number(limit["lower_rad"], "lower_rad")
        high = _number(limit["upper_rad"], "upper_rad")
        _number(limit["max_target_rate_rad_s"], "max_target_rate_rad_s", positive=True)
        if not 0 <= low < high or not low <= baseline[slot].q <= high:
            raise ValueError(f"hand baseline outside approved range at slot {slot}")
    timing = {name: _number(spec[name], name, positive=True) for name in (
        "move_duration_s", "settle_duration_s", "max_feedback_gap_s",
        "max_status_gap_s", "stationary_dq_rad_s",
    )}
    if timing["move_duration_s"] < PERIOD_S or timing["settle_duration_s"] < PERIOD_S:
        raise ValueError("move and settle durations must cover at least one command period")
    if (timing["move_duration_s"] + timing["settle_duration_s"]) / PERIOD_S > 1_000_000:
        raise ValueError("test exceeds one million command periods; split the plan")
    targets = spec["target_positions_rad"]
    if not isinstance(targets, dict) or not targets or not set(targets) <= set(limits):
        raise ValueError("targets must select only configured hand slots")
    parsed_targets = []
    for key, value in targets.items():
        slot = int(key)
        value = _number(value, f"target slot {slot}")
        limit = limits[key]
        if not limit["lower_rad"] <= value <= limit["upper_rad"]:
            raise ValueError(f"target outside approved range at slot {slot}")
        if abs(value - baseline[slot].q) / timing["move_duration_s"] > limit["max_target_rate_rad_s"]:
            raise ValueError(f"target slew exceeds approved rate at slot {slot}")
        parsed_targets.append((slot, value))
    return Plan(tuple(baseline), tuple(sorted(parsed_targets)), tolerance,
                sha256=hashlib.sha256(data).hexdigest(), **timing)


def command_at(plan: Plan, alpha: float, cmd_id: int) -> LowCmd:
    if not math.isfinite(alpha) or not 0 <= alpha <= 1:
        raise ValueError("trajectory alpha must be in [0, 1]")
    motors = list(plan.baseline)
    for slot, target in plan.targets:
        start = motors[slot].q
        motors[slot] = replace(motors[slot], q=start + (target - start) * alpha)
    return LowCmd(cmd_id=cmd_id, motor_cmds=tuple(motors))


def _validate_state(state):
    if not state.attitude_valid or len(state.motor_states) != STATE_SLOTS:
        raise RuntimeError("requires valid full-body attitude and 44 LowState slots")
    if state.stamp_ns <= 0:
        raise RuntimeError("LowState source timestamp is missing")
    for motor in state.motor_states:
        if not math.isfinite(motor.q) or not math.isfinite(motor.dq):
            raise RuntimeError("non-finite LowState q/dq")


def _fresh_state(controller, previous, budget, *, monotonic, sleep):
    deadline = monotonic() + budget
    while monotonic() < deadline:
        state = controller.wait_lowstate(max(0.001, deadline - monotonic()))
        if monotonic() >= deadline:
            raise RuntimeError("LowState wait exceeded the approved feedback budget")
        _validate_state(state)
        if previous is None:
            return state
        if state.state_id > previous.state_id and state.stamp_ns > previous.stamp_ns:
            return state
        if state.state_id != previous.state_id or state.stamp_ns != previous.stamp_ns:
            raise RuntimeError("LowState sequence/time reversed or inconsistent; do not resume")
        sleep(min(0.005, max(0, deadline - monotonic())))
    raise RuntimeError("LowState stopped advancing; no further targets sent")


def _check_pose(state, command, plan, *, stationary=False):
    selected = {slot for slot, _ in plan.targets}
    for slot, (motor, target, tolerance) in enumerate(zip(
            state.motor_states, command.motor_cmds, plan.tolerances)):
        if abs(motor.q - target.q) > tolerance:
            raise RuntimeError(f"slot {slot} feedback outside approved tracking/baseline tolerance")
        if (stationary or slot not in selected) and abs(motor.dq) > plan.stationary_dq_rad_s:
            raise RuntimeError(f"slot {slot} is not stationary")


def run(controller, plan: Plan, *, motion=False, timeout=5.0,
        sleep=time.sleep, monotonic=time.monotonic, emit=print):
    """Observe by default. Caller must separately authorize any motion=True."""
    _number(timeout, "timeout", positive=True)
    if timeout <= PREFLIGHT_WINDOW_S:
        raise ValueError("timeout must exceed the preflight window")
    if type(motion) is not bool:
        raise ValueError("motion must be a boolean")
    def log(event, **fields):
        emit({"event": event, "host_monotonic_s": monotonic(), **fields})

    sampled_at = None

    def sample(previous, budget):
        nonlocal sampled_at
        state = _fresh_state(controller, previous, budget, monotonic=monotonic, sleep=sleep)
        sampled_at = monotonic()
        log("lowstate", state_id=state.state_id, source_stamp_ns=state.stamp_ns,
            q=[motor.q for motor in state.motor_states],
            dq=[motor.dq for motor in state.motor_states])
        if monotonic() - sampled_at >= plan.max_feedback_gap_s:
            raise RuntimeError("LowState became stale while recording evidence")
        return state

    log("preflight_start", spec_sha256=plan.sha256, motion=motion,
        selected_slots=[slot for slot, _ in plan.targets],
        per_motor_freshness="not_available_in_current_sdk",
        physical_stop="not_verified_by_this_program")
    deadline = monotonic() + timeout
    state = None
    window_start = None
    baseline = command_at(plan, 0, 1)
    while True:
        if monotonic() >= deadline:
            raise RuntimeError("preflight timed out")
        state = sample(state, min(plan.max_feedback_gap_s, deadline - monotonic()))
        _check_pose(state, baseline, plan, stationary=True)
        if window_start is None:
            window_start = monotonic()
        elif monotonic() - window_start >= PREFLIGHT_WINDOW_S:
            break
        sleep(PERIOD_S)
    log("preflight_complete", control_requested=False)
    if not motion:
        return
    if monotonic() - sampled_at >= plan.max_feedback_gap_s:
        raise RuntimeError("preflight feedback became stale before requesting control")

    # Entry may have sent PREPARE/COMMIT before throwing. Always attempt to
    # verify cancellation after an entry attempt, not just after its success.
    original_error = None
    try:
        entered = controller.enter_external(timeout)
        last_status = entered
        status_updated_at = monotonic()
        started = monotonic()
        next_publish = started
        previous_command = baseline
        steps = math.ceil(plan.move_duration_s / PERIOD_S)
        settle_steps = math.ceil(plan.settle_duration_s / PERIOD_S)
        motion_deadline = (started + (steps + settle_steps) * PERIOD_S
                           + plan.max_feedback_gap_s)
        for step in range(steps + settle_steps + 1):
            sleep(max(0.0, next_publish - monotonic()))
            state = sample(state, plan.max_feedback_gap_s)
            if monotonic() >= motion_deadline:
                raise RuntimeError("trajectory exceeded its wall-clock budget; do not resume")
            _check_pose(state, previous_command, plan)
            command = command_at(plan, min(step / steps, 1.0), step + 1)
            # This is an SDK call attempt, not wire capture or actuator ACK.
            log("command_attempt", cmd_id=command.cmd_id,
                motor_cmds=[asdict(motor) for motor in command.motor_cmds])
            # Synchronous evidence output may block. Recheck before sending.
            if (monotonic() >= motion_deadline
                    or monotonic() - sampled_at >= plan.max_feedback_gap_s):
                raise RuntimeError("command evidence exceeded feedback/trajectory budget; not sent")
            status = controller.status
            if (status is None or status.current_action != "EXTERNAL"
                    or status.phase != ExternalPhase.RUNNING or status.timeout_hold
                    or (status.generation, status.client_id, status.lease_id)
                    != (entered.generation, entered.client_id, entered.lease_id)
                    or status.reject_count != entered.reject_count
                    or status.ingress_drop_count != entered.ingress_drop_count):
                raise RuntimeError("EXTERNAL ownership/phase or rejection/drop counters changed")
            if status.status_seq > last_status.status_seq:
                last_status, status_updated_at = status, monotonic()
            elif status.status_seq < last_status.status_seq:
                raise RuntimeError("ControlStatus sequence reversed")
            elif monotonic() - status_updated_at >= plan.max_status_gap_s:
                raise RuntimeError("ControlStatus stopped advancing")
            controller.publish(command)
            log("command_returned", cmd_id=command.cmd_id)
            previous_command = command
            next_publish = monotonic() + PERIOD_S  # Never burst to catch up.
        _check_pose(state, command_at(plan, 1, 0), plan, stationary=True)
        log("trajectory_complete", elapsed_s=monotonic() - started,
            physical_stop="not_verified_by_this_program")
    except BaseException as error:
        original_error = error
        raise
    finally:
        # Evidence sink failure must not prevent CANCEL; preserve the original
        # motion exception if both the trajectory and cancellation fail.
        exit_event = {"event": "cancel_result", "result": "UNCONFIRMED"}
        cleanup_error = None
        try:
            status = controller.exit_external(timeout)
            exit_event["current_action"] = status.current_action
            exit_event["phase"] = status.phase.value
            if status.current_action != "PASSIVE" or status.phase != ExternalPhase.IDLE:
                raise RuntimeError("CANCEL did not confirm PASSIVE/IDLE; attendant must verify")
            exit_event["result"] = "PASSIVE_CONFIRMED_NOT_PHYSICAL_STOP"
        except BaseException as error:
            cleanup_error = error
            exit_event["error"] = f"{type(error).__name__}: {error}"
            if original_error is None:
                raise
        finally:
            exit_event["original_error"] = (
                None if original_error is None else f"{type(original_error).__name__}: {original_error}")
            try:
                emit(exit_event)
            except BaseException:
                if original_error is None and cleanup_error is None:
                    raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", required=True, type=Path, help="reviewed complete 42-slot test spec")
    parser.add_argument("--mode", choices=("validate", "observe", "move"), default="validate")
    parser.add_argument("--group", default="default")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--confirm-motion", action="store_true",
                        help="acknowledgement only; not a substitute for current on-site authorization")
    args = parser.parse_args(argv)
    try:
        plan = load_plan(args.spec)
        _number(args.timeout, "timeout", positive=True)
        if args.timeout <= PREFLIGHT_WINDOW_S:
            raise ValueError("timeout must exceed the preflight window")
        if args.mode == "move" and not args.confirm_motion:
            raise ValueError("--mode move requires --confirm-motion and separate on-site approval")
        if args.mode != "move" and args.confirm_motion:
            raise ValueError("--confirm-motion is only valid with --mode move")
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    def emit(event):
        print(json.dumps(event, allow_nan=False), flush=True)
    if args.mode == "validate":
        emit({"event": "offline_validation", "spec_sha256": plan.sha256,
              "selected_slots": [slot for slot, _ in plan.targets],
              "robot_connected": False, "device_limits_verified": False})
        return
    with ExternalController(group=args.group) as controller:
        run(controller, plan, motion=args.mode == "move", timeout=args.timeout, emit=emit)


if __name__ == "__main__":
    main()
