"""Synthetic T034 exercise; no robot, model, lease transport or CAN hardware."""
from dataclasses import replace
import signal
import unittest
from unittest import mock

from body_fixtures import robot_info_reply
from examples.external_control import external_control as body
from examples.fixed_stand_pose.fixed_stand_pose import FixedStandPose
from examples.end_effector.end_effector_external import run_external
from examples.end_effector import end_effector_external as gripper_demo
from locomotion_aorta import BODY_CMD_TOPIC
from test_examples import FakeClock, RecordingController
from test_end_effector_external import Endpoint
from test_teleop_policy import NAMES


def body_info():
    info = replace(robot_info_reply().info.body, joint_names=NAMES, roster_version=1)
    caps = list(info.joints)
    for i in (4, 5, 10, 11):
        caps[i] = replace(caps[i], valid_limits=1,
                          q_min=-.751 if i in (4, 10) else -.43625,
                          q_max=.414 if i in (4, 10) else .43625,
                          dq_min=0., dq_max=0., tau_min=0., tau_max=0.,
                          kp_min=0., kp_max=0., kd_min=0., kd_max=0.)
    return replace(info, joints=tuple(caps))


class ZeroPoseTest(unittest.TestCase):
    def controller(self, clock=None, info=None, **options):
        clock = clock or FakeClock()
        c = RecordingController(clock, **options)
        self.addCleanup(c.close)
        reply = robot_info_reply()
        selected = info or body_info()
        c.transport.get_robot_info = lambda timeout: replace(
            reply, runtime_id=100, info=replace(reply.info, body=selected))
        return c, clock

    def run_zero(self, c, clock, loop, events=None):
        return run_external(c, loop, gripper_demo._MotionOptions(body_pose='zero', timeout=1), body=body,
                            sleep=clock.sleep, monotonic=clock.monotonic,
                            emit=(events if events is not None else []).append)

    def commands(self, c):
        return [value for topic, value in c.transport.published if topic == BODY_CMD_TOPIC]

    def test_frozen_gain_mapping_and_adapter_q_only_limits(self):
        info = body_info()
        self.assertEqual((info.joints[4].supported_fields, info.joints[4].valid_limits,
                          info.joints[4].kp_min, info.joints[4].kp_max), (31, 1, 0., 0.))
        plan = FixedStandPose(info)
        self.assertEqual(plan.profile['robot_source_head'], 'fda136e91b47e6771026043ae69a8b48b756122e')
        gains = plan.profile['joint_gains']
        self.assertEqual(gains['left_shoulder_pitch_joint']['kp'], 60)
        self.assertEqual(gains['left_elbow_joint']['kp'], 40)
        self.assertEqual(gains['right_wrist_pitch_joint']['kd'], 4)
        self.assertEqual(gains['neck_pitch_joint']['kp'], 20)

    def test_target_mock_is_rejected_but_non_target_slot_is_retained(self):
        info = body_info()
        plan = FixedStandPose(info)
        self.assertEqual(len(plan.targets), 30)
        self.assertTrue(info.joints[14].is_mock)
        self.assertNotIn(info.joint_names[14], plan.targets)
        caps = list(info.joints)
        caps[0] = replace(caps[0], is_mock=True)
        self.assertNotEqual(caps[0].supported_fields, 0)
        with self.assertRaisesRegex(ValueError, 'targets unavailable'):
            FixedStandPose(replace(info, joints=tuple(caps)))

    def test_zero_dwell_cannot_succeed_without_a_sample(self):
        events = []
        with mock.patch.object(body, 'DWELL_S', 0), self.assertRaisesRegex(ValueError, 'at least one'):
            gripper_demo._observe_zero_dwell(lambda _: self.fail('must not request feedback'),
                (0.,) * 31, body=body, body_info=body_info(), monotonic=lambda: 0., emit=events.append)
        self.assertEqual(events, [])

    def test_measured_hold_accepts_positive_range_but_zero_mode_rejects(self):
        info = body_info()
        caps = list(info.joints)
        caps[0] = replace(caps[0], q_min=.2, q_max=1.2)
        info = replace(info, joints=tuple(caps))
        c, clock = self.controller(info=info)
        c.measured_q[0] = .7
        run_external(c, lambda frame: frame(), gripper_demo._MotionOptions(body_pose='measured', timeout=1),
                     body=body, sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        self.assertTrue(self.commands(c))
        self.assertEqual(c.calls.count('exit_external'), 1)
        c2, clock2 = self.controller(info=info)
        c2.measured_q[0] = .7
        with self.assertRaisesRegex(ValueError, 'zero outside'):
            run_external(c2, lambda frame: self.fail('zero mode must not enter'),
                         gripper_demo._MotionOptions(body_pose='zero', timeout=1), body=body, sleep=clock2.sleep,
                         monotonic=clock2.monotonic, emit=lambda _: None)
        self.assertNotIn('enter_external', c2.calls)
        self.assertFalse(self.commands(c2))

    def test_native_transition_duration_and_actual_time_curve(self):
        plan = FixedStandPose(body_info())
        for q, expected in ((.01, 0.), (.05, 0.), (.06, .05), (.6, .5)):
            start = (q,) + (0.,) * 30
            self.assertAlmostEqual(plan.duration(start), expected)
        start = (.3,) * 31
        # Public14 is intentionally ignored, even if its placeholder is nonfinite.
        start = start[:14] + (float('nan'),) + start[15:]
        for elapsed, expected in ((0, .3), (.025, .27), (.125, .15), (.25, 0.), (1, 0.)):
            q = plan.position(start, elapsed, .25)
            self.assertTrue(all(v == 0 if i == 14 else abs(v-expected) < 1e-12
                                for i, v in enumerate(q)))
            plan.check(q)

    def test_bad_zero_targets_list_all_joints_before_prepare(self):
        info = body_info()
        caps = list(info.joints)
        caps[3] = replace(caps[3], q_min=.01)
        caps[9] = replace(caps[9], q_max=-.01)
        c, clock = self.controller(info=replace(info, joints=tuple(caps)))
        with self.assertRaises(ValueError) as caught:
            self.run_zero(c, clock, lambda f: self.fail('loop must not run'))
        self.assertIn('left_knee_joint', str(caught.exception))
        self.assertIn('right_knee_joint', str(caught.exception))
        self.assertNotIn('enter_external', c.calls)
        self.assertFalse(self.commands(c))

    def test_missing_q_range_or_start_outside_info_rejected_before_prepare(self):
        for missing in (False, True):
            info = body_info()
            caps = list(info.joints)
            caps[0] = replace(caps[0], valid_limits=0) if missing else replace(caps[0], q_max=.01)
            c, clock = self.controller(info=replace(info, joints=tuple(caps)))
            with self.assertRaises(ValueError if missing else RuntimeError):
                self.run_zero(c, clock, lambda f: self.fail('must not enter'))
            self.assertNotIn('enter_external', c.calls)

    def test_latest_measured_start_zero_endpoint_placeholders_and_cancel(self):
        c, clock = self.controller()
        entered = c.enter_external
        def enter(timeout):
            status = entered(timeout)
            c.measured_q = [.2] * 31
            c.measured_q[14] = float('nan')
            return status
        c.enter_external = enter
        events = []
        self.run_zero(c, clock, lambda frame: frame(), events)
        commands = self.commands(c)
        self.assertTrue(all(j.q == .2 for i, j in enumerate(commands[0].joints) if i != 14))
        self.assertTrue(all(j.q == 0 for j in commands[-1].joints))
        self.assertEqual([m.sequence for m in commands], list(range(1, len(commands)+1)))
        for cmd in commands:
            self.assertEqual((cmd.joints[14].q, cmd.joints[14].dq, cmd.joints[14].tau,
                              cmd.joints[14].kp, cmd.joints[14].kd), (0.,)*5)
            self.assertTrue(all(m.dq == m.tau == 0 for m in cmd.joints))
            FixedStandPose(body_info()).check([m.q for m in cmd.joints])
        self.assertEqual(c.calls.count('enter_external'), 1)
        self.assertEqual(c.calls.count('exit_external'), 1)
        self.assertTrue(any('observation only' in e for e in events))
        self.assertTrue(any('confirmed PASSIVE' in e for e in events))
        self.assertTrue(any('CONTROL' in e.upper() and 'ACK' in e.upper() for e in events))

    def test_settle_zero_nonfollowing_or_drift_is_observation_and_gripper_still_runs(self):
        for options in ({'follow_commands': False},
                        {'drift_command_range': (8, 60), 'drift_position_rad': .2}):
            with self.subTest(options=options):
                c, clock = self.controller(**options)
                gripper = Endpoint(c)
                events = []
                publish = gripper.publish
                def gripper_publish(*args, **kwargs):
                    self.assertNotIn('exit_external', c.calls)
                    self.assertTrue(any('ZERO_DWELL complete observation_only=true' in e for e in events))
                    return publish(*args, **kwargs)
                gripper.publish = gripper_publish
                target = gripper_demo.position_target(gripper.info, (.075,), unit='m')
                gripper_demo.run(c, [gripper], {gripper.side: target},
                                 gripper_demo._MotionOptions(duration=.08, body_pose='zero', timeout=1,
                                     native_hz={gripper.side: 250}, native_config='synthetic-test-effective-config'),
                                 sleep=clock.sleep, monotonic=clock.monotonic, emit=events.append)
                self.assertTrue(gripper.commands)
                complete = [e for e in events if 'ZERO_DWELL complete observation_only=true' in e]
                self.assertEqual(len(complete), 1)
                peak = float(complete[0].split('peak_abs_q_error_rad=')[1].split()[0])
                self.assertGreater(peak, .08)  # Deliberately beyond the removed example-only gate.
                self.assertEqual(c.calls.count('exit_external'), 1)
                self.assertTrue(any('CANCEL confirmed PASSIVE' in e for e in events))

    def test_slow_pc_does_not_stretch_ramp_by_frame_count(self):
        class SlowClock(FakeClock):
            def sleep(self, duration):
                super().sleep(duration + .03)
        c, clock = self.controller(clock=SlowClock())
        self.run_zero(c, clock, lambda frame: frame())
        commands = self.commands(c)
        began = c.command_times[0]
        duration = .1 / 1.2
        for cmd, at in zip(commands, c.command_times):
            expected = .1 * (1. - min(1., max(0., (at-began)/duration)))
            self.assertAlmostEqual(cmd.joints[0].q, expected)

    def test_sigterm_and_rejection_reuse_cancel_without_retry(self):
        c, clock = self.controller()
        def interrupt(frame):
            frame()
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        with self.assertRaises(KeyboardInterrupt):
            self.run_zero(c, clock, interrupt)
        self.assertEqual(c.calls.count('exit_external'), 1)
        c, clock = self.controller(reject_after_commands=2)
        with self.assertRaisesRegex(RuntimeError, 'reject_count'):
            self.run_zero(c, clock, lambda frame: self.fail('gripper loop must not run'))
        self.assertEqual(c.calls.count('exit_external'), 1)

    def test_zero_pose_round_trip_keeps_one_external_session(self):
        c, clock = self.controller()
        g = Endpoint(c)
        initial = g.state.pos[0]
        publish = g.publish
        def checked_publish(*args, **kwargs):
            self.assertTrue(all(j.q == 0 for j in self.commands(c)[-1].joints))
            return publish(*args, **kwargs)
        g.publish = checked_publish
        target = gripper_demo.position_target(g.info, (.075,), unit='m')
        gripper_demo.run(c, [g], {g.side: target},
                         gripper_demo._MotionOptions(duration=.08, body_pose='zero', return_to_start=True, timeout=1,
                             native_hz={g.side: 250}, native_config='synthetic-test-effective-config'),
                         sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        positions = [cmd.target.pos[0] for cmd in g.commands]
        target_indices = [i for i, value in enumerate(positions) if abs(value - .075) < 1e-7]
        self.assertTrue(target_indices)
        target_index = target_indices[0]
        self.assertGreater(target_index, 0)
        self.assertLess(target_index, len(positions) - 1)
        self.assertAlmostEqual(positions[target_index], .075, places=7)
        self.assertAlmostEqual(positions[-1], initial, places=7)
        self.assertTrue(all(a <= b + 1e-7 for a, b in zip(positions[:target_index], positions[1:target_index+1])))
        self.assertTrue(all(a >= b - 1e-7 for a, b in zip(positions[target_index:], positions[target_index+1:])))
        for first, second, start, end in zip(g.commands, g.commands[1:], g.times, g.times[1:]):
            self.assertLessEqual(abs(second.target.pos[0] - first.target.pos[0]), min(end - start, 1/250) + 1e-8)
        self.assertEqual(c.calls.count('enter_external'), 1)
        self.assertEqual(c.calls.count('exit_external'), 1)


if __name__ == '__main__':
    unittest.main()
