"""400 Hz EE snapshots do not drive the demos' 50 Hz command timer."""
from dataclasses import replace
import unittest

from examples.brainco_hand import brainco_hand_external as hand_demo
from examples.gripper import gripper_external as gripper_demo
from test_brainco_hand_external import FakeHand
from test_gripper_external import FakeGripper
from test_demo_pull_policy import Latest400HzController
from test_examples import FakeClock


class EECommandRateTest(unittest.TestCase):
    def test_hand_state_400hz_keeps_hand_and_body_commands_at_50hz(self):
        clock, times, ids = FakeClock(), [], []
        class Hand(FakeHand):
            @property
            def state(self):
                return replace(super().state, state_id=1+round(clock.now*400))
            def publish(self, q, dq):
                times.append(clock.now)
                ids.append(self.state.state_id)
                return super().publish(q, dq)
        body = Latest400HzController(clock)
        self.addCleanup(body.close)
        hand_demo.run(body, [Hand()], timeout=1, speed_deg_s=1, sleep=clock.sleep,
                      monotonic=clock.monotonic, emit=lambda _: None)
        self.assertGreater(len(times), 4)
        for a, b in zip(times, times[1:]):
            # The existing hand sequence has an intentional dwell between legs.
            self.assertGreaterEqual(b-a+1e-12, .02)
        self.assertTrue(any(abs(b-a-.02) < 1e-12 for a, b in zip(times, times[1:])))
        for a, b, ia, ib in zip(times, times[1:], ids, ids[1:]):
            self.assertAlmostEqual((b-a)/.02, round((b-a)/.02))
            self.assertEqual(ib-ia, round((b-a)*400))

    def test_gripper_state_400hz_keeps_gripper_and_body_commands_at_50hz(self):
        clock, times, ids = FakeClock(), [], []
        class Gripper(FakeGripper):
            @property
            def state(self):
                return replace(super().state, state_id=1+round(clock.now*400))
            def publish_target(self, *args, **kwargs):
                times.append(clock.now)
                ids.append(kwargs['input_state'].state_id)
                return super().publish_target(*args, **kwargs)
        body = Latest400HzController(clock)
        self.addCleanup(body.close)
        gripper_demo.run(body, Gripper(), q=.075, duration=.08, timeout=1,
                         sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        self.assertGreater(len(times), 4)
        for a, b in zip(times, times[1:]):
            self.assertAlmostEqual(b-a, .02)
        self.assertEqual([b-a for a, b in zip(ids, ids[1:])], [8]*(len(ids)-1))
