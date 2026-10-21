#!/usr/bin/env python3
"""Velocity mode over SDK 0.2.0, with TONIGHT/071 per-cycle latency records."""

import argparse
from dataclasses import dataclass
import hashlib
from importlib.metadata import version
from locomotion_aorta._demo_feedback import publish_log_fields
import os
from pathlib import Path
import threading
import time
import uuid

from locomotion_aorta import (
    PullController as ExternalController, PullTransport, BodyCmd, BodyJointCmd, JoyReader,
)
import locomotion_aorta
from locomotion_aorta._body_codec import decode_body_state
from locomotion_aorta.transport import (
    _AortaTransport, _load_aorta_bindings,
    BODY_STATE_TOPIC, CONTROL_STATUS_TOPIC,
)

try:
    from .velocity_mode_latency import CycleLog, NOT_MEASURED, realtime_ns
    from .velocity_mode_policy import VelocityModePolicy
except ImportError:
    # Isolated (-I) bundle execution omits the script folder from sys.path.
    import importlib.util
    def _sibling(name):
        spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    _latency = _sibling("velocity_mode_latency")
    CycleLog, NOT_MEASURED, realtime_ns = _latency.CycleLog, _latency.NOT_MEASURED, _latency.realtime_ns
    VelocityModePolicy = _sibling("velocity_mode_policy").VelocityModePolicy


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


def decode_state(view, s1: int, pull_context_receive_stamp_ns=0) -> StateFrame:
    state = decode_body_state(view)
    hip = next(imu for imu in state.imu_states if imu.location == 0)
    q, dq = tuple(m.q for m in state.joints), tuple(m.dq for m in state.joints)
    return StateFrame(state, q, dq, hip.state.gyroscope, hip.state.quaternion,
                      state.ts_state_pub_us * 1000 if state.ts_state_pub_us else NOT_MEASURED,
                      s1, realtime_ns(), pull_context_receive_stamp_ns)


class PolicyTransport(PullTransport):
    """Use SDK wait_state/native NumPy feedback; shadow constructs no publishers."""

    def __init__(self, *, group: str, shadow: bool) -> None:
        self.condition = threading.Condition()
        self.frame = None
        self.observed_status = None
        self._pull_timeout = 5.0
        super().__init__(node_name=f"velocity_mode-{'shadow' if shadow else 'live'}-{uuid.uuid4()}",
                         group=group, publishers=not shadow)

    def _recv_frame(self, timeout):
        state = self.wait_state(timeout)
        hip = next(i for i, location in enumerate(state.imu_location) if location == 0)
        if not state.imu_state_present[hip]:
            raise ValueError('HIP IMU state is absent')
        values = state.imu_values[hip]
        frame = StateFrame(state, state.q, state.dq, values[4:7], values[:4],
            state.ts_state_pub_us * 1000 if state.ts_state_pub_us else NOT_MEASURED,
            state.received_realtime_ns, realtime_ns(), state.pull_context_receive_stamp_ns)
        self.frame = frame
        return frame

    def record_status(self, status):
        self.observed_status = status

    def wait_ready(self, timeout: float) -> StateFrame:
        if self.observed_status is not None and self.observed_status.current_action == "EXTERNAL":
            raise RuntimeError("an EXTERNAL session is already active")
        self._pull_timeout = timeout
        deadline = time.monotonic() + timeout
        self._recv_frame(max(0, deadline - time.monotonic()))
        self.poll_control_status(max(0, deadline - time.monotonic()))
        if self.observed_status is None:
            raise TimeoutError("waiting for body state and ControlStatus")
        if self.observed_status.current_action == "EXTERNAL":
            raise RuntimeError("an EXTERNAL session is already active")
        return self.frame

    def latest(self) -> StateFrame:
        self.poll_control_status()
        frame = self._recv_frame(self._pull_timeout)
        self.poll_control_status()
        return frame


class DirectionInput:
    """Profile adapter; JoyReader owns draining, edge counts and receipt time."""

    def __init__(self, transport):
        self.reader = JoyReader(transport.node)
        self.last = None

    def snapshot(self):
        self.last = self.reader.read()
        return self.last.axes if self.last.is_fresh(realtime_ns()) else ()

    def statistics(self):
        if self.last is None:
            return dict(joy_messages=0, last_joy_ns=None, last_joy_axes=())
        sample = self.last
        return dict(joy_messages=sample.samples_total,
                    last_joy_ns=sample.received_realtime_ns, last_joy_axes=sample.axes,
                    last_joy_buttons=sample.buttons,
                    joy_pressed_counts=dict(sample.pressed_counts),
                    joy_released_counts=dict(sample.released_counts),
                    joy_missing_since_read=sample.missing_samples_since_read,
                    joy_missing_total=sample.missing_samples_total,
                    joy_sequence_available=sample.sequence_available,
                    joy_event_history_complete=sample.event_history_complete,
                    joy_sequence_diagnostics=dict(sample.sequence_diagnostics))

    def close(self):
        self.reader.close()


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


def run_cycles(policy, transport, directions, log, *, controller, duration, transition):
    initial = transport.latest()
    target, transition_s = transition
    policy.reset()
    started = realtime_ns()
    duration_ns = int(duration * 1_000_000_000) if duration else None
    transition_ns = int(transition_s * 1_000_000_000)
    next_cycle = started
    cycle = 0
    previous_publish_ns = None
    while duration_ns is None or realtime_ns() - started < duration_ns:
        delay = next_cycle - realtime_ns()
        if delay > 0:
            time.sleep(delay / 1_000_000_000)
        frame = transport.latest()
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
            axes = directions.snapshot()
            record.update(directions.statistics())
            elapsed = realtime_ns() - started
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
            if controller:
                record["S5"] = realtime_ns()
                publish_mono = time.monotonic_ns()
                record['control_interval_ns'] = (publish_mono-previous_publish_ns
                    if previous_publish_ns is not None else NOT_MEASURED)
                previous_publish_ns = publish_mono
                try:
                    sent = controller.publish(command)
                    publish_log_fields(record, sent)
                    record["publish_result"] = "returned"
                except BaseException:
                    record["publish_result"] = "raised"
                    raise
                finally:
                    record["S6"] = realtime_ns()
                    timing = getattr(controller, '_last_publish_timing', lambda: None)()
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
        # Fixed 50 Hz target; an overrun resumes at the current time without
        # bursts of catch-up commands. No per-segment extra 20 ms delay.
        next_cycle = max(next_cycle + PERIOD_NS, realtime_ns())


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
    metadata = dict(sdk_release="0.2.1", sdk_base_commit="8ef2e015830bc861e293f944f831c1ed05b1136b",
                    sdk_version=version("locomotion-aorta"),
                    sdk_directory=str(Path(locomotion_aorta.__file__).resolve().parent),
                    sdk_files_sha256={name: hashlib.sha256(Path(locomotion_aorta.__file__).with_name(name).read_bytes()).hexdigest()
                                      for name in ("controller.py", "transport.py", "types.py", "_body_codec.py", "__init__.py")},
                    example_directory=str(Path(__file__).resolve().parent),
                    example_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                    for name in ("velocity_mode.py", "velocity_mode_policy.py", "velocity_mode_latency.py")},
                    aorta_sdk_version=version("aorta-sdk"), aorta_msgs_version=version("aorta-msgs"),
                    locomotion_exec_sha256=args.exec_sha256, locomotion_yaml_sha256=args.yaml_sha256,
                    external_pd=args.external_pd, load_description=args.load_description,
                    mode="shadow" if args.shadow else "live", joy_source="aorta",
                    duration_s=args.duration,
                    S1_source="Python pull recv return, before root lookup and decode",
                    S5_source="would-publish boundary" if args.shadow else "ExternalController.publish entry",
                    period_s=PERIOD_S, models=[policy.inventory])
    log = transport = controller = directions = None
    entered = False
    try:
        transport = PolicyTransport(group=args.group, shadow=args.shadow)
        if args.shadow:
            transport.subscribe(CONTROL_STATUS_TOPIC, transport.record_status)
        else:
            controller = ExternalController(group=args.group, _transport=transport)
            transport.subscribe(CONTROL_STATUS_TOPIC, transport.record_status)
        transport.wait_ready(args.timeout)
        response = controller.get_robot_info(args.timeout) if controller else transport.get_robot_info(args.timeout)
        if response.info is None or response.info.body is None:
            raise RuntimeError(response.error_message or "RobotInfo has no body roster")
        policy.bind_body_info(response.info.body)
        log = CycleLog(args.log, metadata)
        log.write(dict(record_type="roster", joint_names=policy.order,
                       roster_version=response.info.body.roster_version,
                       populated=[j.populated for j in response.info.body.joints],
                       is_mock=[j.is_mock for j in response.info.body.joints]))
        log.write(dict(record_type="preflight", timestamp_ns=realtime_ns(),
                       current_action=transport.observed_status.current_action,
                       phase=transport.observed_status.phase.value,
                       generation=transport.observed_status.generation))
        directions = DirectionInput(transport)
        log.write(dict(record_type="transport", timestamp_ns=realtime_ns(),
                       aorta_publishers=len(transport.publishers)))
        if controller:
            controller.enter_external(args.timeout)
            entered = True
        transition = transition_duration(policy, transport.latest(), args.locomotion_dir)
        run_cycles(policy, transport, directions, log, controller=controller,
                   duration=args.duration, transition=transition)
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
                                       source="aorta", **directions.statistics()))
                    finally:
                        directions.close()
            finally:
                if controller is not None:
                    controller.close()
                elif transport is not None:
                    transport.close()
                if log is not None:
                    log.close()


if __name__ == "__main__":
    main()
