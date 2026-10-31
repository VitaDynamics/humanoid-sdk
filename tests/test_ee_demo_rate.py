"""400 Hz actual EE SampleRefs do not drive the single CLI's 50 Hz timers."""
from dataclasses import replace
import unittest

from examples.end_effector import end_effector_external as demo
from examples.external_control import external_control
from locomotion_aorta import BODY_CMD_TOPIC
from test_end_effector_external import Endpoint
from test_demo_pull_policy import Latest400HzController
from test_examples import FakeClock


class EECommandRateTest(unittest.TestCase):
    def check_rates(self, hand):
        clock = FakeClock()
        class LatestEndpoint(Endpoint):
            @property
            def state(self):
                return replace(super().state, state_id=1+round(clock.now*400))
        body = Latest400HzController(clock)
        self.addCleanup(body.close)
        endpoint = LatestEndpoint(body, hand=hand)
        target = demo.position_target(endpoint.info, (.0625,)*len(endpoint.info.axis_names),
                                      unit='rad' if hand else 'm')
        # Synthetic native cadence is independent of the 50Hz publication timer.
        native = {} if hand else {
            'native_hz': {endpoint.side: 250},
            'native_config': 'synthetic-test-effective-config',
        }
        demo.run(body, [endpoint], {endpoint.side: target},
                 demo._MotionOptions(duration=.12, timeout=1, **native),
                 sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None)
        body_commands = [message for topic, message in body.transport.published
                         if topic == BODY_CMD_TOPIC]
        self.assertGreaterEqual(len(body_commands), 4)
        self.assertEqual(len(body.command_times), len(body_commands))
        for start, end in zip(body.command_times, body.command_times[1:]):
            self.assertGreaterEqual(end-start+1e-12, external_control.PERIOD_S)
        self.assertAlmostEqual(body.command_times[3]-body.command_times[0],
                               3*external_control.PERIOD_S)
        self.assertEqual([after.input_state_id-before.input_state_id
                          for before, after in zip(body_commands[:3], body_commands[1:4])],
                         [8]*3)
        self.assertGreater(len(endpoint.times), 4)
        for times in (endpoint.times, body.command_times):
            for a, b in zip(times, times[1:]): self.assertAlmostEqual(b-a, .02)
        ids = [command.input_observations[0].sample_id for command in endpoint.commands]
        self.assertEqual([b-a for a, b in zip(ids, ids[1:])], [8]*(len(ids)-1))
        self.assertEqual(body.calls.count('exit_external'), 1)

    def test_hand_state_400hz_keeps_hand_and_body_commands_at_50hz(self):
        self.check_rates(True)

    def test_gripper_state_400hz_keeps_gripper_and_body_commands_at_50hz(self):
        self.check_rates(False)
