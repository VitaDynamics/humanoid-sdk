#!/usr/bin/env python3
"""Velocity mode over SDK 0.2.1 with per-cycle latency records."""

import argparse
from dataclasses import dataclass
from importlib.metadata import version
import os
import signal
import sys
from pathlib import Path
import threading
import time
import uuid

from locomotion_aorta import (
    PullController, PullStateSource, PullTransport, BodyCmd, BodyJointCmd, JoyReader, ExternalPhase,
)
import locomotion_aorta


try:
    from .velocity_mode_policy import VelocityModePolicy
except ImportError:
    # Isolated (-I) bundle execution does not put the script folder on sys.path.
    import importlib.util
    _policy_spec = importlib.util.spec_from_file_location(
        "velocity_mode_policy", Path(__file__).with_name("velocity_mode_policy.py"))
    _policy = importlib.util.module_from_spec(_policy_spec)
    _policy_spec.loader.exec_module(_policy)
    VelocityModePolicy = _policy.VelocityModePolicy


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from policy_support import (CycleLog, NOT_MEASURED, realtime_ns, publish_log_fields,
                            LOWSTATE_WINDOW_S, joy_log_fields)


PERIOD_S = 0.02
PERIOD_NS = int(PERIOD_S * 1_000_000_000)


@dataclass(frozen=True)
class StateFrame:
    state: object
    q: tuple
    dq: tuple
    gyro: tuple
    quat: tuple
    s0: int | str
    s1: int
    s2: int
    pull_context_receive_stamp_ns: int = 0


def read_frame(transport, timeout):
    state = transport.wait_state(timeout)
    hip = next(i for i, location in enumerate(state.imu_location) if location == 0)
    if not state.imu_state_present[hip]:
        raise ValueError('HIP IMU state is absent')
    values = state.imu_values[hip]
    return StateFrame(state, state.q, state.dq, values[4:7], values[:4],
        state.ts_state_pub_us * 1000 if state.ts_state_pub_us else NOT_MEASURED,
        state.received_realtime_ns, realtime_ns(), state.pull_context_receive_stamp_ns)


def wait_ready(transport, timeout):
    if transport.control_status is not None and transport.control_status.current_action == "EXTERNAL":
        raise RuntimeError("an EXTERNAL session is already active")
    deadline = time.monotonic() + timeout
    read_frame(transport, max(0, deadline - time.monotonic()))
    status = transport.poll_control_status(max(0, deadline - time.monotonic()))
    if status is None:
        raise TimeoutError("waiting for body state and ControlStatus")
    if status.current_action == "EXTERNAL":
        raise RuntimeError("an EXTERNAL session is already active")
    return status


def transition_duration(policy, frame, root):
    import numpy as np
    import yaml

    config = yaml.safe_load((root / "config/locomotion_foot_humanoid_asample.yaml").read_text())
    cfg = config["transition"]
    params = config["actions"]["HUMANOID"]["params"]
    target = policy.decode(np.zeros(len(policy.names), dtype=np.float32))
    delta = max(abs(frame.q[i] - target[i]["q"]) for i in policy.driven_slots)
    threshold = params["transition_small_delta_skip_threshold_q"]
    duration = (0.0 if delta <= threshold else
                min(max(delta / params["transition_speed_q_per_s"], cfg["min_duration_s"]),
                    cfg["max_duration_s"]))
    return target, duration


def require_running(controller, expected=None):
    status = controller.status
    if (status is None or status.current_action != 'EXTERNAL'
            or status.phase != ExternalPhase.RUNNING or getattr(status, 'timeout_hold', False)):
        raise RuntimeError('EXTERNAL is no longer RUNNING')
    if expected is not None and any(getattr(status, key, 0) != getattr(expected, key, 0)
            for key in ('generation', 'client_id', 'control_id', 'reject_count', 'ingress_drop_count')):
        raise RuntimeError('EXTERNAL ownership or rejection/drop counters changed')
    return status


def run_cycles(policy, frames, directions, log, *, controller, duration, transition, stop=None):
    target_names = tuple(name for i, name in enumerate(policy.order) if i in policy.driven_slots)
    if not target_names:
        raise ValueError('velocity policy has no declared Body targets')
    required_fields = 0x1b | (0x04 if policy.config.get('ankle_clamp_feedforward', False) else 0)
    initial = frames()
    target, transition_s = transition
    policy.reset()
    started = time.monotonic_ns()
    stop = stop if stop is not None else threading.Event()
    expected = require_running(controller) if controller else None
    last_state_id, last_advance = initial.state.state_id, started
    duration_ns = int(duration * 1_000_000_000) if duration else None
    transition_ns = int(transition_s * 1_000_000_000)
    next_cycle = started
    cycle = 0
    previous_publish_ns = None
    while not stop.is_set() and (duration_ns is None or time.monotonic_ns() - started < duration_ns):
        delay = next_cycle - time.monotonic_ns()
        if delay > 0:
            time.sleep(delay / 1_000_000_000)
        if stop.is_set():
            break
        frame = frames()
        now = time.monotonic_ns()
        if frame.state.state_id > last_state_id:
            last_state_id, last_advance = frame.state.state_id, now
        if now-last_advance >= LOWSTATE_WINDOW_S*1e9:
            raise TimeoutError('BodyState did not advance within the live lowstate window')
        if controller:
            require_running(controller, expected)
        cycle += 1
        record = dict(cycle=cycle, state_id=frame.state.state_id,
                      S0=frame.s0, S1=frame.s1, S2=frame.s2,
                      S3=NOT_MEASURED, S4=NOT_MEASURED, S5=NOT_MEASURED,
                      S6=NOT_MEASURED, L1=NOT_MEASURED, L2=NOT_MEASURED,
                      sequence=NOT_MEASURED,
                      session_id=NOT_MEASURED, publish_result="not_called",
                      phase="policy", mode="live" if controller else "shadow",
                      applied_session_id=frame.state.applied_session_id,
                      applied_sequence=frame.state.applied_sequence,
                      ts_first_output_us=frame.state.ts_first_output_us,
                      joint_error_codes=(frame.state.error_code.tolist() if hasattr(frame.state, "error_code")
                                         else [j.error_code for j in frame.state.joints]))
        if getattr(frame, 'pull_context_receive_stamp_ns', 0):
            record['pull_context_receive_stamp_ns'] = frame.pull_context_receive_stamp_ns
        try:
            # Drain even during transition so queued short presses are consumed
            # on every 50 Hz tick, without changing the policy's timer.
            sample = directions.read()
            axes = sample.axes if sample.is_fresh(realtime_ns()) else ()
            record.update(joy_log_fields(sample))
            elapsed = time.monotonic_ns() - started
            if elapsed < transition_ns:
                record["phase"] = "transition"
                progress = elapsed / transition_ns
                motors = [dict(motor, q=initial.q[i] + (motor["q"] - initial.q[i]) * progress)
                          if i in policy.driven_slots else dict(motor)
                          for i, motor in enumerate(target)]
            else:
                command = policy.command(axes)
                observation = policy.observe(frame.q, frame.dq, frame.gyro, frame.quat, command)
                record["S3"] = realtime_ns()
                action = policy.infer(observation)
                record["S4"] = realtime_ns()
                motors = policy.decode(action)
                record["velocity_command"] = command.tolist()
                record["raw_action"] = action.tolist()
            command = BodyCmd(joints=tuple(BodyJointCmd(**m) for m in motors),
                              input_state_id=frame.state.state_id,
                              input_state_ts_real_us=frame.state.ts_state_real_us)
            record["target_q"] = [motor.q for motor in command.joints]
            if stop.is_set():
                break
            if controller:
                require_running(controller, expected)
                if time.monotonic_ns()-last_advance >= LOWSTATE_WINDOW_S*1e9:
                    raise TimeoutError('BodyState did not advance within the live lowstate window')
                record["S5"] = realtime_ns()
                publish_mono = time.monotonic_ns()
                record['control_interval_ns'] = (publish_mono-previous_publish_ns
                    if previous_publish_ns is not None else NOT_MEASURED)
                previous_publish_ns = publish_mono
                try:
                    sent = controller.publish(command, target_names=target_names, required_fields=required_fields)
                    publish_log_fields(record, sent)
                    record["publish_result"] = "returned"
                except BaseException:
                    record["publish_result"] = "raised"
                    raise
                finally:
                    record["S6"] = realtime_ns()
                    timing = getattr(controller, 'publish_timing', None)
                    if timing is not None:
                        record['publish_timing'] = timing
            else:
                record["S5"] = realtime_ns()
                record["publish_result"] = "shadow_would_publish"
                record["candidate_cycle"] = cycle
        except BaseException as error:
            record["error"] = f"{type(error).__name__}: {error}"
            raise
        finally:
            log.cycle(record)
        # Keep a monotonic 50 Hz timer; after an overrun wait a full period.
        next_deadline = next_cycle + PERIOD_NS
        now = time.monotonic_ns()
        next_cycle = next_deadline if next_deadline > now else now + PERIOD_NS


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--shadow", action="store_true")
    modes.add_argument("--live", action="store_true")
    parser.add_argument("--locomotion-dir", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--group", default="default")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="total loop seconds; 0 or omitted runs until interrupted")
    parser.add_argument("--exec-sha256", default=NOT_MEASURED)
    parser.add_argument("--yaml-sha256", default=NOT_MEASURED)
    parser.add_argument("--external-pd", choices=("PD-A", "PD-B", NOT_MEASURED), default=NOT_MEASURED)
    parser.add_argument("--load-description", default=NOT_MEASURED)
    args = parser.parse_args(argv)
    if not 0 <= args.duration < float("inf"):
        parser.error("--duration must be finite and non-negative (0 means unlimited)")
    return args


def main():
    args = parse_args()
    policy = VelocityModePolicy(args.locomotion_dir)
    metadata = dict(sdk_version=version("locomotion-aorta"),
                    sdk_directory=str(Path(locomotion_aorta.__file__).resolve().parent),
                    example_directory=str(Path(__file__).resolve().parent),
                    aorta_sdk_version=version("aorta-sdk"), aorta_msgs_version=version("aorta-msgs"),
                    locomotion_exec_sha256=args.exec_sha256, locomotion_yaml_sha256=args.yaml_sha256,
                    external_pd=args.external_pd, load_description=args.load_description,
                    mode="shadow" if args.shadow else "live", joy_source="aorta",
                    duration_s=args.duration,
                    S1_source="Python pull recv return, before root lookup and decode",
                    S5_source="would-publish boundary" if args.shadow else "ExternalController.publish entry",
                    period_s=PERIOD_S, models=[policy.inventory])
    log = transport = controller = source = directions = None
    entered = False
    stop = threading.Event()
    handlers = {sig: signal.signal(sig, lambda *_args: stop.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        transport = PullTransport(group=args.group, publishers=not args.shadow,
            node_name=f"velocity-mode-{uuid.uuid4().hex}")
        if not args.shadow:
            controller = PullController(group=args.group, _transport=transport)
        else:
            source = PullStateSource(group=args.group, _transport=transport)
        reader = controller if controller is not None else source
        response = reader.get_robot_info(args.timeout)
        if response.info is None or response.info.body is None:
            raise RuntimeError(response.error_message or "RobotInfo has no body roster")
        status = wait_ready(transport, args.timeout)
        frames = lambda: read_frame(reader, args.timeout)
        policy.bind_body_info(response.info.body)
        from locomotion_aorta._policy_state import format_robot_info
        print(format_robot_info(response))
        log = CycleLog(args.log, metadata)
        log.write(dict(record_type="roster", joint_names=policy.order,
                       roster_version=response.info.body.roster_version,
                       is_mock=[j.is_mock for j in response.info.body.joints],
                       supported_fields=[j.supported_fields for j in response.info.body.joints],
                       valid_limits=[j.valid_limits for j in response.info.body.joints]))
        log.write(dict(record_type="preflight", timestamp_ns=realtime_ns(),
                       current_action=status.current_action,
                       phase=status.phase.value,
                       generation=status.generation))
        directions = JoyReader(transport.node)
        log.write(dict(record_type="transport", timestamp_ns=realtime_ns(),
                       aorta_publishers=len(transport.publishers)))
        if controller:
            controller.enter_external(args.timeout)
            entered = True
        transition = transition_duration(policy, frames(), args.locomotion_dir)
        run_cycles(policy, frames, directions, log, controller=controller,
                   duration=args.duration, transition=transition, stop=stop)
    finally:
        try:
            if entered:
                status = controller.exit_external(args.timeout)
                log.write(dict(record_type="cancel", current_action=status.current_action,
                               phase=status.phase.value, timestamp_ns=realtime_ns()))
        finally:
            try:
                if directions is not None:
                    try:
                        log.write(dict(record_type="direction_input", timestamp_ns=realtime_ns(),
                                       source="aorta", **joy_log_fields(directions.read())))
                    finally:
                        directions.close()
            finally:
                try:
                    if controller is not None:
                        controller.close()
                    elif source is not None:
                        source.close()
                    elif transport is not None:
                        transport.close()
                finally:
                    try:
                        if log is not None:
                            log.close()
                    finally:
                        for sig, handler in handlers.items():
                            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
