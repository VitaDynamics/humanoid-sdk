"""Synthetic T034 exercise; no robot, model, lease transport or CAN hardware."""
from dataclasses import replace
import signal
import unittest

from body_fixtures import robot_info_reply
from examples import external_control as body
from examples.fixed_stand_pose import FixedStandPose
from examples.gripper_external import run_external
from examples import gripper_external as gripper_demo
from locomotion_aorta import BODY_CMD_TOPIC
from test_examples import FakeClock, RecordingController
from test_gripper_external import FakeGripper
from test_teleop_policy import NAMES


def body_info():
    info = replace(robot_info_reply().info.body, joint_names=NAMES, roster_version=1)
    caps = list(info.joints)
    for i in (4, 5, 10, 11):
        caps[i] = replace(caps[i], valid_limits=1,
                          q_min=-.751 if i in (4, 10) else -.43625,
                          q_max=.414 if i in (4, 10) else .43625)
    return replace(info, joints=tuple(caps))


class ZeroPoseTest(unittest.TestCase):
    def controller(self, clock=None, info=None, **options):
        clock = clock or FakeClock()
        c = RecordingController(clock, **options)
        self.addCleanup(c.close)
        reply = robot_info_reply()
        c.get_robot_info = lambda timeout: replace(reply, info=replace(reply.info, body=info or body_info()))
        return c, clock

    def run_zero(self, c, clock, loop, events=None):
        return run_external(c, loop, body=body, body_pose='zero', timeout=1,
                            sleep=clock.sleep, monotonic=clock.monotonic,
                            emit=(events if events is not None else []).append)

    def commands(self, c):
        return [value for topic, value in c.transport.published if topic == BODY_CMD_TOPIC]

    def test_frozen_gain_mapping_and_adapter_q_only_limits(self):
        plan = FixedStandPose(body_info())
        self.assertEqual(plan.profile['robot_source_head'], 'fda136e91b47e6771026043ae69a8b48b756122e')
        gains = plan.profile['joint_gains']
        self.assertEqual(gains['left_shoulder_pitch_joint']['kp'], 60)
        self.assertEqual(gains['left_elbow_joint']['kp'], 40)
        self.assertEqual(gains['right_wrist_pitch_joint']['kd'], 4)
        self.assertEqual(gains['neck_pitch_joint']['kp'], 20)

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
            with self.assertRaises(ValueError):
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
                gripper = FakeGripper()
                events = []
                publish = gripper.publish_target
                def gripper_publish(*args, **kwargs):
                    self.assertNotIn('exit_external', c.calls)
                    self.assertTrue(any('ZERO_DWELL complete observation_only=true' in e for e in events))
                    return publish(*args, **kwargs)
                gripper.publish_target = gripper_publish
                gripper_demo.run(c, gripper, q=.075, duration=.08, body_pose='zero',
                                 timeout=1, sleep=clock.sleep, monotonic=clock.monotonic, emit=events.append)
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
        g = FakeGripper()
        initial = g.state.sample.q
        publish = g.publish_target
        def checked_publish(*args, **kwargs):
            self.assertTrue(all(j.q == 0 for j in self.commands(c)[-1].joints))
            return publish(*args, **kwargs)
        g.publish_target = checked_publish
        gripper_demo.run(c, g, q=.075, duration=.08, body_pose='zero', return_to_start=True,
                         timeout=1, sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        self.assertEqual(list(dict.fromkeys(cmd.q for cmd in g.commands)), [.075, initial])
        self.assertEqual(c.calls.count('enter_external'), 1)
        self.assertEqual(c.calls.count('exit_external'), 1)


if __name__ == '__main__':
    unittest.main()
