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
    def run_with_work(self, *, body_enabled=True, overrun=False, right_hz=50.):
        clock=FakeClock()
        controller=Latest400HzController(clock)
        self.addCleanup(controller.close)
        original=controller.publish
        def publish_body(*args,**kwargs):
            sent=original(*args,**kwargs)
            clock.sleep(.002)
            return sent
        controller.publish=publish_body
        endpoints=[Endpoint(controller,side,hand=True) for side in ('left','right')]
        stalled=False
        def endpoint_work(_sent):
            nonlocal stalled
            if overrun and not stalled:
                stalled=True
                clock.sleep(.055)
            else:
                clock.sleep(.002)
        for endpoint in endpoints:endpoint.after_publish=endpoint_work
        targets={e.side:demo.position_target(e.info,(.0625,)*6,unit='rad') for e in endpoints}
        demo.run(controller,endpoints,targets,
                 demo._MotionOptions(duration=.32,timeout=1.,body_enabled=body_enabled,
                    rates={'left':50.,'right':right_hz}),
                 sleep=clock.sleep,monotonic=clock.monotonic,emit=lambda _:None)
        self.assertEqual(1,controller.calls.count('enter_external'))
        self.assertEqual(1,controller.calls.count('exit_external'))
        return controller,endpoints

    def test_body_and_both_endpoints_keep_50hz_despite_publication_work(self):
        controller,endpoints=self.run_with_work()
        for times in (controller.command_times,*(e.times for e in endpoints)):
            self.assertGreater(len(times),10)
            for a,b in zip(times,times[1:]):self.assertAlmostEqual(.020,b-a,places=9)

    def test_ee_only_independent_rates_do_not_accumulate_work_time(self):
        controller,endpoints=self.run_with_work(body_enabled=False,right_hz=25.)
        self.assertEqual([],controller.command_times)
        for endpoint,period in zip(endpoints,(.020,.040)):
            self.assertAlmostEqual(period,(endpoint.times[-1]-endpoint.times[0])/(len(endpoint.times)-1),places=9)
            # Two independent timers occasionally collide; the second caller
            # can wait for the first 2ms call without accumulating that delay.
            for i,stamp in enumerate(endpoint.times):
                self.assertLessEqual(abs(stamp-endpoint.times[0]-i*period),.002+1e-9)

    def test_overrun_does_not_replay_all_missed_publications(self):
        _,endpoints=self.run_with_work(body_enabled=False,overrun=True)
        left,right=endpoints
        self.assertGreater(left.times[1]-left.times[0],.055)
        # Once the first long call ends, each endpoint resumes its timer rather
        # than emitting the missed 20/40ms points in an immediate burst.
        for endpoint in (left,right):
            intervals=[b-a for a,b in zip(endpoint.times,endpoint.times[1:])]
            self.assertTrue(all(value>=.020-1e-9 for value in intervals))
            self.assertTrue(all(abs(value-.020)<1e-9 for value in intervals[2:]))

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
