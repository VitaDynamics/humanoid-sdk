import unittest

from examples import external_control, lowstate_subscriber
from locomotion_aorta import (
    CONTROL_ACK_TOPIC,
    CONTROL_REQUEST_TOPIC,
    CONTROL_STATUS_TOPIC,
    EXTERNAL_COMMAND_TOPIC,
    LOWSTATE_TOPIC,
    ControlAck,
    ControlOperation,
    ControlStatus,
    ExternalController,
    ExternalPhase,
    HumanoidLowState,
    MotorState,
)
from fake_transport import FakeTransport


class RecordingController:
    def __init__(
        self,
        clock: "FakeClock",
        *,
        lease_remaining_s: float = 90.0,
        advance_lowstate: bool = True,
        cancel_action: str = "PASSIVE",
        fail_phase_after_commands: int | None = None,
        reject_after_commands: int | None = None,
        stale_after_commands: int | None = None,
        drift_command_range: tuple[int, int] | None = None,
        drift_position_rad: float = 0.2,
        follow_commands: bool = True,
    ) -> None:
        self.clock = clock
        self.lease_remaining_s = lease_remaining_s
        self.advance_lowstate = advance_lowstate
        self.cancel_action = cancel_action
        self.fail_phase_after_commands = fail_phase_after_commands
        self.reject_after_commands = reject_after_commands
        self.stale_after_commands = stale_after_commands
        self.drift_command_range = drift_command_range
        self.drift_position_rad = drift_position_rad
        self.follow_commands = follow_commands
        self.transport = FakeTransport()
        self.controller = ExternalController(
            client_id="example-client", _transport=self.transport
        )
        self.calls: list[str] = []
        self.status_sequence = 0
        self.state_id = 6
        self.measured_q = [0.1] * 42
        self.measured_dq = [0.0] * 42
        self.attitude_valid = True
        self.command_count = 0
        self.command_times: list[float] = []
        self.transport.on_publish = self._on_publish
        self._emit_lowstate()

    @property
    def status(self):
        return self.controller.status

    def wait_lowstate(self, timeout: float):
        self.calls.append("wait_lowstate")
        self._emit_lowstate()
        return self.controller.wait_lowstate(timeout)

    def enter_external(self, timeout: float, *, lease_duration_s: float = 30.0):
        self.calls.append("enter_external")
        return self.controller.enter_external(
            timeout, lease_duration_s=lease_duration_s
        )

    def publish(self, command):
        self.calls.append("publish")
        self.controller.publish(command)
        if self.follow_commands:
            self.measured_q = [motor.q for motor in command.motor_cmds]

    def exit_external(self, timeout: float):
        self.calls.append("exit_external")
        return self.controller.exit_external(timeout)

    def close(self) -> None:
        self.controller.close()

    def _emit_lowstate(self) -> None:
        if self.advance_lowstate and not (
            self.stale_after_commands is not None
            and self.command_count >= self.stale_after_commands
        ):
            self.state_id += 1
        emitted_q = self.measured_q
        if (
            self.drift_command_range is not None
            and self.drift_command_range[0]
            <= self.command_count
            < self.drift_command_range[1]
        ):
            emitted_q = [q + self.drift_position_rad for q in emitted_q]
        self.transport.emit(
            LOWSTATE_TOPIC,
            HumanoidLowState(
                state_id=self.state_id,
                stamp_ns=int(self.clock.monotonic() * 1_000_000_000),
                motor_states=tuple(
                    MotorState(q=q, dq=dq)
                    for q, dq in zip(emitted_q, self.measured_dq)
                ),
                attitude_valid=self.attitude_valid,
            ),
        )

    def _on_publish(self, topic, message) -> None:
        if topic == EXTERNAL_COMMAND_TOPIC:
            self.command_count += 1
            self.command_times.append(self.clock.monotonic())
            phase = ExternalPhase.RUNNING
            reject_count = 0
            if (
                self.fail_phase_after_commands is not None
                and self.command_count >= self.fail_phase_after_commands
            ):
                phase = ExternalPhase.TIMEOUT_HOLD
            if (
                self.reject_after_commands is not None
                and self.command_count >= self.reject_after_commands
            ):
                reject_count = 1
            if phase != ExternalPhase.RUNNING or reject_count:
                self.status_sequence += 1
                self.transport.emit(
                    CONTROL_STATUS_TOPIC,
                    ControlStatus(
                        status_seq=self.status_sequence,
                        current_action="EXTERNAL",
                        phase=phase,
                        client_id="example-client",
                        lease_id="lease-example",
                        lease_deadline_monotonic_ns=31_000_000_000,
                        generation=17,
                        reject_count=reject_count,
                        stamp_monotonic_ns=(
                            1_000_000_000 + self.command_count
                        ),
                    ),
                )
            return
        if topic != CONTROL_REQUEST_TOPIC:
            return
        if message.operation == ControlOperation.PREPARE:
            self.transport.emit(
                CONTROL_ACK_TOPIC,
                ControlAck(
                    request_id=message.request_id,
                    operation=message.operation,
                    accepted=True,
                    lease_id="lease-example",
                ),
            )
        elif message.operation == ControlOperation.COMMIT:
            self.transport.emit(
                CONTROL_ACK_TOPIC,
                ControlAck(
                    request_id=message.request_id,
                    operation=message.operation,
                    accepted=True,
                    lease_id="lease-example",
                ),
            )
            self.status_sequence += 1
            self.transport.emit(
                CONTROL_STATUS_TOPIC,
                ControlStatus(
                    status_seq=self.status_sequence,
                    current_action="EXTERNAL",
                    phase=ExternalPhase.RUNNING,
                    client_id="example-client",
                    lease_id="lease-example",
                    lease_deadline_monotonic_ns=int(
                        (1.0 + self.lease_remaining_s) * 1_000_000_000
                    ),
                    generation=17,
                    stamp_monotonic_ns=1_000_000_000,
                ),
            )
        elif message.operation == ControlOperation.CANCEL:
            self.transport.emit(
                CONTROL_ACK_TOPIC,
                ControlAck(
                    request_id=message.request_id,
                    operation=message.operation,
                    accepted=True,
                    lease_id="lease-example",
                ),
            )
            self.status_sequence += 1
            self.transport.emit(
                CONTROL_STATUS_TOPIC,
                ControlStatus(
                    status_seq=self.status_sequence,
                    current_action=self.cancel_action,
                    phase=ExternalPhase.IDLE,
                    stamp_monotonic_ns=2_000_000_000,
                ),
            )


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class ExamplesSmokeTest(unittest.TestCase):
    def test_lowstate_example_prints_snapshot_without_entering_external(self):
        harness = RecordingController(FakeClock())
        self.addCleanup(harness.close)
        output: list[str] = []

        lowstate_subscriber.run_once(
            harness, timeout=0.1, emit=output.append
        )

        self.assertEqual(harness.calls, ["wait_lowstate"])
        self.assertEqual(
            output,
            ["state_id=8 stamp_ns=0"]
            + [
                f"joint[{index}] q=0.100000 dq=0.000000"
                for index in range(42)
            ],
        )

    def test_external_example_runs_full_body_round_trip_and_enters_passive(self):
        clock = FakeClock()
        harness = RecordingController(clock)
        self.addCleanup(harness.close)
        output: list[str] = []

        external_control.run(
            harness,
            timeout=1.0,
            sleep=clock.sleep,
            monotonic=clock.monotonic,
            emit=output.append,
        )

        self.assertEqual(harness.calls[0], "wait_lowstate")
        self.assertIn("enter_external", harness.calls)
        self.assertEqual(harness.calls[-1], "exit_external")
        commands = [
            message
            for topic, message in harness.transport.published
            if topic == EXTERNAL_COMMAND_TOPIC
        ]
        self.assertEqual(len(commands), 301)
        self.assertEqual(
            [command.external_sequence for command in commands],
            list(range(1, 302)),
        )
        self.assertEqual(len(commands[0].motor_cmds), 42)
        self.assertTrue(
            all(motor.q == 0.1 for motor in commands[0].motor_cmds)
        )
        self.assertTrue(
            all(motor.q == 0.0 for motor in commands[100].motor_cmds)
        )
        for slot, motor in enumerate(commands[250].motor_cmds):
            self.assertEqual(motor.q, 0.0 if slot in (17, 21) else 0.1)
        max_step = max(
            abs(current.motor_cmds[0].q - previous.motor_cmds[0].q)
            for previous, current in zip(commands, commands[1:])
        )
        self.assertLessEqual(max_step, 0.002)
        self.assertEqual(commands[0].motor_cmds[0].kp, 100.0)
        self.assertEqual(commands[0].motor_cmds[25].kp, 0.0)
        self.assertEqual(commands[0].motor_cmds[25].kd, 0.0)
        for command in commands:
            for slot in (14, 17, 18, 21):
                self.assertEqual(command.motor_cmds[slot].kp, 80.0)
                self.assertEqual(
                    command.motor_cmds[slot].kd,
                    1.0 if slot in (17, 21) else 2.0,
                )
            for slot in (15, 16, 19, 20):
                self.assertEqual(command.motor_cmds[slot].kp, 40.0)
        self.assertEqual(output[-1], "CANCEL confirmed PASSIVE")

    def test_return_offsets_only_elbows_toward_zero_without_crossing_zero(self):
        for left, right, expected_left, expected_right in (
            (0.3, -0.3, 0.125467074800567, -0.125467074800567),
            (-0.3, 0.3, -0.125467074800567, 0.125467074800567),
            (0.17453292519943295, -0.17453292519943295, 0.0, 0.0),
            (0.1, -0.1, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0),
        ):
            with self.subTest(left=left, right=right):
                clock = FakeClock()
                harness = RecordingController(clock)
                self.addCleanup(harness.close)
                harness.measured_q[17] = left
                harness.measured_q[21] = right
                external_control.run(
                    harness, timeout=1.0, sleep=clock.sleep,
                    monotonic=clock.monotonic, emit=lambda _: None,
                )
                commands = [
                    message for topic, message in harness.transport.published
                    if topic == EXTERNAL_COMMAND_TOPIC
                ]
                for slot, motor in enumerate(commands[-1].motor_cmds):
                    expected = {17: expected_left, 21: expected_right}.get(
                        slot, 0.1
                    )
                    self.assertAlmostEqual(motor.q, expected)
                for slot, start in ((17, left), (21, right)):
                    positions = [
                        command.motor_cmds[slot].q for command in commands
                    ]
                    self.assertGreaterEqual(min(positions), min(start, 0.0))
                    self.assertLessEqual(max(positions), max(start, 0.0))
                    self.assertLessEqual(
                        max(abs(b - a) for a, b in zip(positions, positions[1:])),
                        0.002,
                    )
                self.assertEqual(harness.status.current_action, "PASSIVE")

    def test_zero_and_return_have_distinct_position_tolerances(self):
        for phase, window, error, accepted in (
            ("zero", (101, 151), 0.06, True),
            ("zero", (101, 151), 0.08, True),
            ("zero", (101, 151), 0.081, False),
            ("return", (251, 301), 0.049, True),
            ("return", (251, 301), 0.051, False),
            ("return", (251, 301), 0.06, False),
        ):
            with self.subTest(phase=phase, error=error):
                clock = FakeClock()
                harness = RecordingController(
                    clock, drift_command_range=window, drift_position_rad=error
                )
                self.addCleanup(harness.close)
                output: list[str] = []
                if accepted:
                    external_control.run(
                        harness, timeout=1.0, sleep=clock.sleep,
                        monotonic=clock.monotonic, emit=output.append,
                    )
                    self.assertIn("return endpoint settled", output)
                else:
                    with self.assertRaisesRegex(
                        RuntimeError, f"{phase} endpoint did not settle"
                    ):
                        external_control.run(
                            harness, timeout=1.0, sleep=clock.sleep,
                            monotonic=clock.monotonic, emit=output.append,
                        )
                    self.assertNotIn("return endpoint settled", output)
                self.assertEqual(harness.status.current_action, "PASSIVE")

    def test_zero_tolerance_does_not_relax_endpoint_velocity_limit(self):
        clock = FakeClock()
        harness = RecordingController(clock)
        self.addCleanup(harness.close)

        def sleep_with_motion(duration: float) -> None:
            clock.sleep(duration)
            if harness.command_count >= 100:
                harness.measured_dq[17] = 0.051

        with self.assertRaisesRegex(RuntimeError, "zero endpoint did not settle"):
            external_control.run(
                harness, timeout=1.0, sleep=sleep_with_motion,
                monotonic=clock.monotonic, emit=lambda _: None,
            )
        self.assertEqual(harness.status.current_action, "PASSIVE")

    def test_external_example_does_not_burst_after_scheduler_stall(self):
        clock = FakeClock()
        harness = RecordingController(clock)
        self.addCleanup(harness.close)
        stalled = False

        def sleep_with_stall(duration: float) -> None:
            nonlocal stalled
            clock.sleep(duration)
            if harness.command_count == 10 and not stalled:
                clock.now += 0.3
                stalled = True

        external_control.run(
            harness,
            timeout=1.0,
            sleep=sleep_with_stall,
            monotonic=clock.monotonic,
            emit=lambda _: None,
        )

        self.assertTrue(stalled)
        self.assertGreater(len(harness.command_times), 1)
        self.assertGreaterEqual(
            min(
                current - previous
                for previous, current in zip(
                    harness.command_times, harness.command_times[1:]
                )
            )
            + 1e-12,
            external_control.PERIOD_S,
        )

    def test_external_example_runs_large_pose_with_90_second_lease(self):
        clock = FakeClock()
        harness = RecordingController(clock)
        self.addCleanup(harness.close)
        harness.measured_q[31] = -1.95
        external_control.run(
            harness, timeout=1.0, sleep=clock.sleep,
            monotonic=clock.monotonic, emit=lambda _: None,
        )
        requests = [
            message for topic, message in harness.transport.published
            if topic == CONTROL_REQUEST_TOPIC
        ]
        self.assertEqual(requests[0].requested_lease_ms, 90_000)
        commands = [
            message for topic, message in harness.transport.published
            if topic == EXTERNAL_COMMAND_TOPIC
        ]
        positions = [command.motor_cmds[31].q for command in commands]
        self.assertEqual(positions[0], -1.95)
        self.assertIn(0.0, positions)
        self.assertEqual(positions[-1], -1.95)
        max_step = max(abs(b - a) for a, b in zip(positions, positions[1:]))
        self.assertLessEqual(max_step, 0.002)
        duration = harness.command_times[-1] - harness.command_times[0]
        self.assertGreater(duration, 60.0)
        self.assertLess(duration, 61.0)
        self.assertEqual(harness.status.current_action, "PASSIVE")

    def test_external_example_rejects_trajectory_that_exceeds_nominal_lease(self):
        clock = FakeClock()
        harness = RecordingController(clock)
        self.addCleanup(harness.close)
        harness.measured_q = [3.0] * 42

        with self.assertRaisesRegex(ValueError, "90-second lease"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertNotIn("enter_external", harness.calls)

    def test_external_example_cancels_when_actual_remaining_lease_is_short(self):
        clock = FakeClock()
        harness = RecordingController(clock, lease_remaining_s=30.0)
        self.addCleanup(harness.close)
        harness.measured_q[31] = -1.95

        with self.assertRaisesRegex(RuntimeError, "remaining lease"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertIn("enter_external", harness.calls)
        self.assertEqual(harness.calls[-1], "exit_external")
        self.assertFalse(
            any(
                topic == EXTERNAL_COMMAND_TOPIC
                for topic, _ in harness.transport.published
            )
        )

    def test_external_example_rejects_stale_preflight_lowstate(self):
        clock = FakeClock()
        harness = RecordingController(clock, advance_lowstate=False)
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "lowstate.*stale"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertNotIn("enter_external", harness.calls)

    def test_external_example_requires_passive_after_cancel(self):
        clock = FakeClock()
        harness = RecordingController(clock, cancel_action="EMERGENCY")
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "expected PASSIVE"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

    def test_external_example_cancels_on_timeout_hold(self):
        clock = FakeClock()
        harness = RecordingController(clock, fail_phase_after_commands=1)
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "no longer.*RUNNING"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertEqual(harness.calls[-1], "exit_external")

    def test_external_example_cancels_when_reject_count_increases(self):
        clock = FakeClock()
        harness = RecordingController(clock, reject_after_commands=1)
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "reject_count changed"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertEqual(harness.calls[-1], "exit_external")

    def test_external_example_cancels_on_stale_running_lowstate(self):
        clock = FakeClock()
        harness = RecordingController(clock, stale_after_commands=1)
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "lowstate became stale"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertEqual(harness.calls[-1], "exit_external")

    def test_external_example_requires_endpoint_settle(self):
        clock = FakeClock()
        harness = RecordingController(clock, follow_commands=False)
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "zero endpoint did not settle"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertEqual(harness.calls[-1], "exit_external")

    def test_external_example_rejects_drift_after_initial_endpoint_settle(self):
        clock = FakeClock()
        harness = RecordingController(
            clock, drift_command_range=(130, 151)
        )
        self.addCleanup(harness.close)

        with self.assertRaisesRegex(RuntimeError, "zero endpoint did not settle"):
            external_control.run(
                harness,
                timeout=1.0,
                sleep=clock.sleep,
                monotonic=clock.monotonic,
                emit=lambda _: None,
            )

        self.assertEqual(harness.calls[-1], "exit_external")

    def test_preflight_waits_for_a_new_continuous_stationary_window(self):
        for spike_start, spike_end, earliest_finish in (
            (0.0, 0.04, 0.24),
            (0.18, 0.24, 0.44),
        ):
            with self.subTest(spike_start=spike_start):
                clock = FakeClock()
                harness = RecordingController(clock)
                self.addCleanup(harness.close)

                def sleep_with_feedback(duration):
                    clock.sleep(duration)
                    harness.measured_dq[31] = (
                        -0.07326 if spike_start <= clock.now < spike_end else 0.0
                    )

                sleep_with_feedback(0)
                state = external_control._wait_for_stationary_lowstate(
                    harness, timeout=5.0, sleep=sleep_with_feedback,
                    monotonic=clock.monotonic,
                )
                self.assertEqual(state.motor_states[31].dq, 0.0)
                self.assertGreaterEqual(clock.now + 1e-12, earliest_finish)
                self.assertLess(clock.now, earliest_finish + 0.06)
                self.assertNotIn("enter_external", harness.calls)

    def test_preflight_continuous_motion_times_out_without_entering(self):
        clock = FakeClock()
        harness = RecordingController(clock)
        self.addCleanup(harness.close)
        harness.measured_dq[31] = 0.051
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            external_control.run(
                harness, timeout=5.0, sleep=clock.sleep,
                monotonic=clock.monotonic, emit=lambda _: None,
            )
        self.assertAlmostEqual(clock.now, 5.0)
        self.assertNotIn("enter_external", harness.calls)
        self.assertEqual(harness.transport.published, [])

    def test_external_example_rejects_moving_or_invalid_preflight(self):
        for invalid_setup, error in (
            (lambda h: setattr(h, "measured_dq", [0.051] * 42), "timed out"),
            (lambda h: setattr(h, "attitude_valid", False), "attitude"),
            (lambda h: setattr(h, "measured_q", [0.0] * 41), "42 required"),
            (lambda h: setattr(h, "measured_q", [float("nan")] * 42), "finite"),
            (lambda h: setattr(h, "measured_dq", [float("nan")] * 42), "finite"),
        ):
            with self.subTest(error=error):
                clock = FakeClock()
                harness = RecordingController(clock)
                self.addCleanup(harness.close)
                invalid_setup(harness)

                with self.assertRaisesRegex(RuntimeError, error):
                    external_control.run(
                        harness,
                        timeout=1.0,
                        sleep=clock.sleep,
                        monotonic=clock.monotonic,
                        emit=lambda _: None,
                    )

                self.assertNotIn("enter_external", harness.calls)
                if error != "timed out":
                    self.assertEqual(clock.now, 0.0)

    def test_preflight_does_not_wait_out_stale_moving_feedback(self):
        clock = FakeClock()
        harness = RecordingController(clock, advance_lowstate=False)
        self.addCleanup(harness.close)
        harness.measured_dq[31] = -0.07326
        with self.assertRaisesRegex(RuntimeError, "stale"):
            external_control.run(
                harness, timeout=5.0, sleep=clock.sleep,
                monotonic=clock.monotonic, emit=lambda _: None,
            )
        self.assertLess(clock.now, 0.3)
        self.assertNotIn("enter_external", harness.calls)


if __name__ == "__main__":
    unittest.main()
