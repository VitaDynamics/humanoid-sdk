#!/usr/bin/env python3
"""Velocity mode over SDK 0.2.0, with TONIGHT/071 per-cycle latency records."""

import argparse
from dataclasses import dataclass
import hashlib
from importlib.metadata import version
import os
from pathlib import Path
import threading
import time
import uuid

from locomotion_aorta import ExternalController, BodyCmd, BodyJointCmd
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
    from velocity_mode_latency import CycleLog, NOT_MEASURED, realtime_ns
    from velocity_mode_policy import VelocityModePolicy


PERIOD_S = 0.02
# RobotStateTimeoutConfig in the source baseline; no new timeout parameter.
INPUT_FRESHNESS_S = 1.0
PERIOD_NS = int(PERIOD_S * 1_000_000_000)
INPUT_FRESHNESS_NS = int(INPUT_FRESHNESS_S * 1_000_000_000)


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


def decode_state(view, s1: int) -> StateFrame:
    state = decode_body_state(view)
    hip = next(imu for imu in state.imu_states if imu.location == 0)
    q, dq = tuple(m.q for m in state.joints), tuple(m.dq for m in state.joints)
    return StateFrame(state, q, dq, hip.state.gyroscope, hip.state.quaternion,
                      state.ts_state_pub_us * 1000 if state.ts_state_pub_us else NOT_MEASURED,
                      s1, realtime_ns())


class PolicyTransport(_AortaTransport):
    """Reuse SDK body codecs/session protocol; shadow constructs no publishers."""

    def __init__(self, *, group: str, shadow: bool) -> None:
        self.condition = threading.Condition()
        self.frame = None
        self.observed_status = None
        self.decode_error = None
        if shadow:
            # Native Aorta otherwise creates its own /log publisher, even
            # when the Python transport never asks for a publisher.
            os.environ["AORTA_LOG_TOPIC_ENABLED"] = "false"
            self._bindings = _load_aorta_bindings()
            self.node = self._bindings.aorta.Node(f"velocity_mode-shadow-{uuid.uuid4()}", group)
            self.publishers = {}
            self.subscriptions = []
            self._closed = False
            self._info_clients = {}
            self._info_lock = threading.Lock()
        else:
            super().__init__(node_name=f"velocity_mode-live-{uuid.uuid4()}", group=group)

    def subscribe(self, topic, callback):
        if topic != BODY_STATE_TOPIC:
            return super().subscribe(topic, callback)

        def decoder(payload):
            # Callback entry before FlatBuffers root lookup or materialization.
            s1 = realtime_ns()
            return self._bindings.body_state_decoder(payload), s1

        def received(decoded, context):
            view, s1 = decoded
            try:
                frame = decode_state(view, s1)
            except Exception as error:
                with self.condition:
                    self.decode_error = error
                    self.condition.notify_all()
                return
            with self.condition:
                self.frame = frame
                self.condition.notify_all()
            callback(frame.state)

        sub = self.node.create_subscriber_typed(
            decoder, topic, received, qos=self._bindings.aorta.QoS.sensor_data()
        )
        self.subscriptions.append(sub)
        return sub

    def record_status(self, status):
        with self.condition:
            self.observed_status = status
            self.condition.notify_all()

    def wait_ready(self, timeout: float) -> StateFrame:
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.decode_error is not None or
                (self.frame is not None and self.observed_status is not None), timeout
            )
            if self.decode_error is not None:
                raise self.decode_error
            if not ready:
                raise TimeoutError("waiting for body state and ControlStatus")
            if self.observed_status.current_action == "EXTERNAL":
                raise RuntimeError("an EXTERNAL session is already active")
            return self.frame

    def latest(self) -> StateFrame:
        with self.condition:
            if self.decode_error is not None:
                raise self.decode_error
            return self.frame


class DirectionInput:
    def __init__(self, transport, source):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import Twist

        if source == "aorta":
            try:
                from locomotion.Joy import Joy as AortaJoy
            except ImportError as error:
                raise RuntimeError("Aorta direction input requires deployment-matched locomotion.Joy bindings; "
                                   "the public SDK wheel set does not include them. "
                                   "Select --joy-source ros2 only when explicitly intended.") from error

        class NoPublisher:
            def publish(self, _message):
                pass

        class SubscriberNode(Node):
            def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
                # rclpy Node otherwise creates /parameter_events implicitly.
                if topic == "/parameter_events":
                    return NoPublisher()
                raise RuntimeError(f"subscriber node cannot publish {topic}")

        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.axes = ()
        self.twist = (0.0, 0.0, 0.0)
        self.joy_time = self.twist_time = None
        self.joy_messages = self.twist_messages = 0
        self.error = None
        rclpy.init(args=[])
        self.node = SubscriberNode(f"velocity_mode_direction_{uuid.uuid4().hex}",
                                   enable_rosout=False, start_parameter_services=False)
        if source == "ros2":
            self.joy_sub = self.node.create_subscription(Joy, "/joy", self.on_joy, 10)
        else:
            self.joy_sub = transport.node.create_subscriber_typed(
                AortaJoy.GetRootAs, "locomotion/joy", self.on_aorta_joy,
                qos=transport._bindings.aorta.QoS.realtime_control()
            )
        self.twist_sub = self.node.create_subscription(Twist, "/vel_cmd", self.on_twist, 10)
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception as error:
            if self.rclpy.ok():
                self.error = error

    def on_joy(self, message):
        with self.lock:
            self.axes = tuple(message.axes)
            self.joy_time = realtime_ns()
            self.joy_messages += 1

    def on_aorta_joy(self, message, context):
        with self.lock:
            self.axes = tuple(message.Axes(i) for i in range(message.AxesLength()))
            self.joy_time = realtime_ns()
            self.joy_messages += 1

    def on_twist(self, message):
        with self.lock:
            self.twist = (message.linear.x, message.linear.y, message.angular.z)
            self.twist_time = realtime_ns()
            self.twist_messages += 1

    def snapshot(self):
        if self.error is not None:
            raise self.error
        with self.lock:
            now = realtime_ns()
            return (self.axes if self.joy_time is not None and
                    0 <= now - self.joy_time <= INPUT_FRESHNESS_NS else (),
                    self.twist if self.twist_time is not None and
                    0 <= now - self.twist_time <= INPUT_FRESHNESS_NS else (0.0, 0.0, 0.0))

    def statistics(self):
        with self.lock:
            return dict(joy_messages=self.joy_messages, twist_messages=self.twist_messages,
                        last_joy_ns=self.joy_time, last_twist_ns=self.twist_time,
                        last_joy_axes=self.axes, last_twist=self.twist)

    def close(self):
        self.rclpy.shutdown()
        self.thread.join()
        self.node.destroy_node()


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
                      joint_error_codes=[j.error_code for j in frame.state.joints])
        try:
            elapsed = realtime_ns() - started
            if elapsed < transition_ns:
                record["phase"] = "transition"
                progress = elapsed / transition_ns
                motors = [dict(motor, q=initial.q[i] + (motor["q"] - initial.q[i]) * progress)
                          if i in policy.driven_slots else dict(motor)
                          for i, motor in enumerate(target)]
            else:
                axes, twist = directions.snapshot()
                command = policy.command(axes, twist)
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
                try:
                    sent = controller.publish(command)
                    record.update(session_id=sent.session_id, sequence=sent.sequence)
                    record["publish_result"] = "returned"
                except BaseException:
                    record["publish_result"] = "raised"
                    raise
                finally:
                    record["S6"] = realtime_ns()
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
    parser.add_argument("--joy-source", choices=("aorta", "ros2"), default="aorta")
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
    metadata = dict(sdk_release="0.2.0", sdk_base_commit="335341bc765f0ad46a2b77eec8abcccd2b22756b",
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
                    mode="shadow" if args.shadow else "live", joy_source=args.joy_source,
                    duration_s=args.duration,
                    S1_source="Python typed decoder entry (includes decoding; excludes pre-dispatch queue)",
                    S5_source="would-publish boundary" if args.shadow else "ExternalController.publish entry",
                    period_s=PERIOD_S, models=[policy.inventory])
    log = transport = controller = directions = None
    entered = False
    try:
        transport = PolicyTransport(group=args.group, shadow=args.shadow)
        if args.shadow:
            transport.subscribe(BODY_STATE_TOPIC, lambda state: None)
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
        directions = DirectionInput(transport, args.joy_source)
        log.write(dict(record_type="transport", timestamp_ns=realtime_ns(),
                       aorta_publishers=len(transport.publishers),
                       ros2_publishers=sum(1 for _ in directions.node.publishers)))
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
                                       source=args.joy_source, **directions.statistics()))
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
