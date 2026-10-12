from dataclasses import replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from examples import brainco_hand_control as demo
from locomotion_aorta import EXTERNAL_COMMAND_TOPIC, ControlOperation, MotorState
from test_examples import FakeClock, RecordingController


def synthetic_spec():
    """Test fixtures only, never a device configuration or motion approval."""
    return {
        "format_version": 1, "robot_profile": "foot_humanoid_asample_brainco",
        "reviewed_by": "synthetic-test-only", "robot_config_sha256": "0" * 64,
        "device_preflight_evidence": "synthetic-not-hardware",
        "stop_method": "synthetic-not-approved",
        "baseline": [dict(mode=10, q=0.1, dq=0, tau=0 if slot in demo.HAND_SLOTS else 0.02,
                          kp=0 if slot in demo.HAND_SLOTS else 17,
                          kd=0 if slot in demo.HAND_SLOTS else 1)
                     for slot in range(42)],
        "hand_limits": {str(slot): dict(lower_rad=0, upper_rad=1, max_target_rate_rad_s=1)
                        for slot in demo.HAND_SLOTS},
        "target_positions_rad": {"25": 0.12, "37": 0.08},
        "position_tolerance_rad": [0.05] * 42,
        "move_duration_s": 0.1, "settle_duration_s": 0.04,
        "max_feedback_gap_s": 0.1, "max_status_gap_s": 10,
        "stationary_dq_rad_s": 0.01,
    }


class HandController(RecordingController):
    """Real SDK control-session logic, fake transport and 44-slot feedback."""
    def __init__(self, clock, **kwargs):
        super().__init__(clock, **kwargs)
        self.cached = None

    def wait_lowstate(self, timeout):
        state = super().wait_lowstate(timeout)
        if self.cached is not None and state.state_id == self.cached.state_id:
            return self.cached
        self.cached = replace(state, stamp_ns=state.stamp_ns + state.state_id,
                              motor_states=state.motor_states + (MotorState(), MotorState()))
        return self.cached


class BrainCoHandControlTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = Path(self.folder.name) / "spec.json"
        self.spec = synthetic_spec()
        self.clock = FakeClock()
        self.events = []

    def plan(self):
        self.path.write_text(json.dumps(self.spec))
        return demo.load_plan(self.path)

    def controller(self, **kwargs):
        controller = HandController(self.clock, **kwargs)
        self.addCleanup(controller.close)
        return controller

    def run_demo(self, controller, **kwargs):
        demo.run(controller, self.plan(), timeout=1, sleep=self.clock.sleep,
                 monotonic=self.clock.monotonic, emit=self.events.append, **kwargs)

    def test_default_cli_is_offline_and_does_not_construct_transport(self):
        self.plan()
        with patch.object(demo, "ExternalController") as controller, patch("sys.stdout", new_callable=io.StringIO) as out:
            demo.main(["--spec", str(self.path)])
        controller.assert_not_called()
        self.assertFalse(json.loads(out.getvalue())["robot_connected"])

    def test_observe_is_readonly_without_enter_or_cancel(self):
        controller = self.controller()
        self.run_demo(controller)
        self.assertEqual(controller.transport.published, [])
        self.assertNotIn("enter_external", controller.calls)
        self.assertNotIn("exit_external", controller.calls)

    def test_move_requires_confirmation_before_transport(self):
        self.plan()
        with patch.object(demo, "ExternalController") as controller, patch("sys.stderr", new_callable=io.StringIO):
            with self.assertRaises(SystemExit):
                demo.main(["--spec", str(self.path), "--mode", "move"])
        controller.assert_not_called()

    def test_only_selected_hand_positions_change_and_all_other_fields_preserved(self):
        controller = self.controller()
        plan = self.plan()
        self.run_demo(controller, motion=True)
        commands = [msg for topic, msg in controller.transport.published if topic == EXTERNAL_COMMAND_TOPIC]
        self.assertGreater(len(commands), 5)
        self.assertEqual(commands[0].motor_cmds, plan.baseline)
        self.assertEqual(commands[-1].motor_cmds[25].q, 0.12)
        self.assertEqual(commands[-1].motor_cmds[37].q, 0.08)
        self.assertEqual([c.external_sequence for c in commands], list(range(1, len(commands) + 1)))
        for command in commands:
            self.assertEqual(len(command.motor_cmds), 42)
            for slot, motor in enumerate(command.motor_cmds):
                if slot in (25, 37):
                    self.assertEqual(replace(motor, q=0.1), plan.baseline[slot])
                else:
                    self.assertEqual(motor, plan.baseline[slot])
        for before, after in zip(controller.command_times, controller.command_times[1:]):
            self.assertGreaterEqual(after - before + 1e-12, demo.PERIOD_S)
        self.assertEqual(controller.calls[-1], "exit_external")
        self.assertEqual(self.events[-1]["result"], "PASSIVE_CONFIRMED_NOT_PHYSICAL_STOP")
        self.assertIn("per_motor_freshness", self.events[0])

    def test_each_of_twelve_hand_slots_is_mapped_without_touching_others(self):
        for selected in demo.HAND_SLOTS:
            self.spec["target_positions_rad"] = {str(selected): 0.13}
            plan = self.plan()
            for alpha in (0, 0.5, 1):
                command = demo.command_at(plan, alpha, 1)
                for slot, motor in enumerate(command.motor_cmds):
                    expected = 0.1 + 0.03 * alpha if slot == selected else 0.1
                    self.assertAlmostEqual(motor.q, expected)

    def test_hand_command_roundtrips_through_real_generated_bindings(self):
        import flatbuffers
        import locomotion_aorta.transport as transport

        bindings = transport._load_aorta_bindings()
        command = replace(demo.command_at(self.plan(), 1, 8), external_version=1,
                          external_session_id=7, external_sequence=9)
        builder = flatbuffers.Builder(2048)
        builder.Finish(transport._fill_low_cmd(bindings, command, builder, 0))
        view = bindings.low_cmd_module.LowCmd.GetRootAs(builder.Output())
        self.assertEqual(view.MotorCmdLength(), 42)
        self.assertEqual((view.CmdId(), view.ExternalVersion(), view.ExternalSessionId(), view.ExternalSequence()),
                         (8, 1, 7, 9))
        for slot, motor in enumerate(command.motor_cmds):
            encoded = view.MotorCmd(slot)
            self.assertEqual(encoded.Mode(), motor.mode)
            for field in ("q", "dq", "tau", "kp", "kd"):
                self.assertAlmostEqual(getattr(encoded, field.capitalize())(), getattr(motor, field), places=6)

    def test_spec_rejects_unsafe_or_ambiguous_inputs(self):
        mutations = [
            lambda s: s["baseline"].pop(),
            lambda s: s["baseline"][0].pop("tau"),
            lambda s: s["baseline"][0].update(dq=1),
            lambda s: s["baseline"][25].update(kp=1),
            lambda s: s["baseline"][25].update(mode=True),
            lambda s: s["baseline"][25].update(q=float("nan")),
            lambda s: s.update(target_positions_rad={"24": 0.1}),
            lambda s: s.update(target_positions_rad={"25": 2}),
            lambda s: s.update(target_positions_rad={}),
            lambda s: s["hand_limits"].pop("39"),
            lambda s: s["hand_limits"]["25"].update(max_target_rate_rad_s=0.001),
            lambda s: s.update(move_duration_s=0),
            lambda s: s.update(max_feedback_gap_s=float("inf")),
            lambda s: s.update(stop_method="TBD"),
            lambda s: s.update(robot_profile="upper_body_bench"),
            lambda s: s.update(position_tolerance_rad=[0.1] * 41),
            lambda s: s.update(robot_config_sha256="missing"),
            lambda s: s.update(extra="typo"),
        ]
        for change in mutations:
            self.spec = synthetic_spec()
            change(self.spec)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.plan()

    def test_duplicate_json_keys_are_rejected(self):
        self.path.write_text('{"format_version":1,"format_version":1}')
        with self.assertRaisesRegex(ValueError, "duplicate"):
            demo.load_plan(self.path)

    def test_stale_preflight_never_requests_control(self):
        controller = self.controller(advance_lowstate=False)
        with self.assertRaisesRegex(RuntimeError, "stopped advancing"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.transport.published, [])

    def test_preflight_rejects_unapproved_pose_motion_and_bad_attitude(self):
        for field, value in (("measured_q", [0.9] * 42), ("measured_dq", [1] * 42),
                             ("attitude_valid", False)):
            controller = self.controller()
            setattr(controller, field, value)
            with self.assertRaises(RuntimeError):
                self.run_demo(controller, motion=True)
            self.assertEqual(controller.transport.published, [])

    def test_stale_running_feedback_stops_targets_and_cancels(self):
        controller = self.controller(stale_after_commands=1)
        with self.assertRaisesRegex(RuntimeError, "stopped advancing"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.command_count, 1)
        self.assertEqual(controller.calls[-1], "exit_external")

    def test_status_timeout_hold_and_reject_counter_abort(self):
        for kwargs in ({"fail_phase_after_commands": 1}, {"reject_after_commands": 1}):
            controller = self.controller(**kwargs)
            with self.assertRaisesRegex(RuntimeError, "ownership/phase"):
                self.run_demo(controller, motion=True)
            self.assertEqual(controller.command_count, 1)
            self.assertEqual(controller.calls[-1], "exit_external")

    def test_stale_control_status_aborts(self):
        self.spec["max_status_gap_s"] = 0.02
        controller = self.controller()
        with self.assertRaisesRegex(RuntimeError, "ControlStatus stopped"):
            self.run_demo(controller, motion=True)
        self.assertLessEqual(controller.command_count, 2)
        self.assertEqual(controller.calls[-1], "exit_external")

    def test_lost_ownership_does_not_resume(self):
        controller = self.controller()
        original = controller.transport.on_publish
        def replace_owner(topic, message):
            original(topic, message)
            if topic == EXTERNAL_COMMAND_TOPIC:
                controller.controller._on_status(replace(controller.status, status_seq=100, client_id="other"))
        controller.transport.on_publish = replace_owner
        with self.assertRaisesRegex(RuntimeError, "ownership/phase"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.command_count, 1)

    def test_keyboard_interrupt_after_publish_attempt_cancels(self):
        controller = self.controller()
        publish = controller.publish
        def interrupted(command):
            publish(command)
            raise KeyboardInterrupt()
        controller.publish = interrupted
        with self.assertRaises(KeyboardInterrupt):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.command_count, 1)
        self.assertEqual(controller.calls[-1], "exit_external")
        self.assertIn("KeyboardInterrupt", self.events[-1]["original_error"])

    def test_partial_entry_failure_still_verifies_cancel(self):
        controller = self.controller()
        enter = controller.enter_external
        def failed(timeout):
            enter(timeout)
            raise RuntimeError("entry lost confirmation")
        controller.enter_external = failed
        with self.assertRaisesRegex(RuntimeError, "entry lost"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.command_count, 0)
        self.assertEqual(controller.calls[-1], "exit_external")
        self.assertEqual(self.events[-1]["result"], "PASSIVE_CONFIRMED_NOT_PHYSICAL_STOP")

    def test_cancel_error_is_unconfirmed_and_never_reported_as_success(self):
        controller = self.controller(cancel_action="EMERGENCY")
        with self.assertRaisesRegex(RuntimeError, "CANCEL did not confirm"):
            self.run_demo(controller, motion=True)
        self.assertEqual(self.events[-1]["result"], "UNCONFIRMED")

    def test_cleanup_failure_preserves_original_exception(self):
        controller = self.controller(cancel_action="EMERGENCY")
        controller.publish = lambda _: (_ for _ in ()).throw(ValueError("original publish failure"))
        with self.assertRaisesRegex(ValueError, "original publish"):
            self.run_demo(controller, motion=True)
        self.assertEqual(self.events[-1]["result"], "UNCONFIRMED")
        self.assertIn("original publish", self.events[-1]["original_error"])

    def test_output_failure_cannot_prevent_cancel(self):
        controller = self.controller()
        def broken_sink(event):
            if event["event"] in ("command_attempt", "cancel_result"):
                raise BrokenPipeError("stdout closed")
        with self.assertRaises(BrokenPipeError):
            demo.run(controller, self.plan(), motion=True, timeout=1,
                     sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=broken_sink)
        self.assertEqual(controller.calls[-1], "exit_external")
        self.assertEqual(controller.status.current_action, "PASSIVE")

    def test_non_selected_joint_drift_aborts_before_next_command(self):
        controller = self.controller(drift_command_range=(1, 999))
        with self.assertRaisesRegex(RuntimeError, "slot 0"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.command_count, 1)
        self.assertEqual(controller.calls[-1], "exit_external")

    def test_no_automatic_return_or_motion_retry(self):
        controller = self.controller()
        self.run_demo(controller, motion=True)
        requests = [message for _, message in controller.transport.published if hasattr(message, "operation")]
        self.assertEqual([r.operation for r in requests],
                         [ControlOperation.PREPARE, ControlOperation.COMMIT, ControlOperation.CANCEL])
        self.assertTrue(all(request.requested_lease_ms == 0 for request in requests))
        self.assertEqual(controller.measured_q[25], 0.12)

    def test_delayed_fresh_feedback_does_not_bypass_time_budget(self):
        controller = self.controller()
        wait = controller.wait_lowstate
        def late(timeout):
            self.clock.sleep(timeout + 0.001)
            return wait(timeout)
        controller.wait_lowstate = late
        with self.assertRaisesRegex(RuntimeError, "feedback budget"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.transport.published, [])

    def test_slow_processing_aborts_instead_of_extending_motion_indefinitely(self):
        controller = self.controller()
        def slow(event):
            self.events.append(event)
            if event["event"] == "command_returned":
                self.clock.sleep(0.05)
        with self.assertRaisesRegex(RuntimeError, "wall-clock budget"):
            demo.run(controller, self.plan(), motion=True, timeout=1,
                     sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=slow)
        self.assertEqual(controller.calls[-1], "exit_external")

    def test_non_selected_joint_velocity_aborts_even_inside_position_tolerance(self):
        controller = self.controller()
        publish = controller.publish
        def move_leg(command):
            publish(command)
            controller.measured_dq[0] = 0.1
        controller.publish = move_leg
        with self.assertRaisesRegex(RuntimeError, "slot 0 is not stationary"):
            self.run_demo(controller, motion=True)
        self.assertEqual(controller.command_count, 1)

    def test_missing_final_evidence_is_not_a_successful_run(self):
        controller = self.controller()
        def failed_final_log(event):
            if event["event"] == "cancel_result":
                raise BrokenPipeError("final log failed")
        with self.assertRaises(BrokenPipeError):
            demo.run(controller, self.plan(), motion=True, timeout=1,
                     sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=failed_final_log)
        self.assertEqual(controller.status.current_action, "PASSIVE")

    def test_slow_preflight_logging_never_requests_control(self):
        for delayed_event in ("lowstate", "preflight_complete"):
            with self.subTest(event=delayed_event):
                controller = self.controller()
                def slow(event):
                    if event["event"] == delayed_event:
                        self.clock.sleep(self.spec["max_feedback_gap_s"] + 0.001)
                with self.assertRaisesRegex(RuntimeError, "stale"):
                    demo.run(controller, self.plan(), motion=True, timeout=1,
                             sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=slow)
                self.assertEqual(controller.transport.published, [])

    def test_slow_command_logging_does_not_send_stale_target(self):
        controller = self.controller()
        def slow(event):
            if event["event"] == "command_attempt":
                self.clock.sleep(self.spec["max_feedback_gap_s"] + 0.001)
        with self.assertRaisesRegex(RuntimeError, "not sent"):
            demo.run(controller, self.plan(), motion=True, timeout=1,
                     sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=slow)
        self.assertEqual(controller.command_count, 0)
        self.assertEqual(controller.status.current_action, "PASSIVE")

    def test_ownership_loss_during_command_logging_prevents_publish(self):
        controller = self.controller()
        def lose_owner(event):
            if event["event"] == "command_attempt":
                controller.controller._on_status(replace(controller.status, status_seq=100, client_id="other"))
        with self.assertRaisesRegex(RuntimeError, "ownership/phase"):
            demo.run(controller, self.plan(), motion=True, timeout=1,
                     sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=lose_owner)
        self.assertEqual(controller.command_count, 0)
        self.assertEqual(controller.calls[-1], "exit_external")


if __name__ == "__main__":
    unittest.main()
