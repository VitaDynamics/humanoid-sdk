"""Offline 400 Hz sources: policy timers must not follow state arrival rate."""
import threading
import unittest
from types import SimpleNamespace as NS

from examples import brainco_hand_external, external_control, gripper_external, teleop_policy
from locomotion_aorta import BODY_CMD_TOPIC
from test_brainco_hand_external import FakeHand
from test_examples import FakeClock, RecordingController
from test_teleop_policy import API, CONTRACT, METADATA, fixture


class Latest400HzController(RecordingController):
    def _emit_body_state(self):
        self.state_id = 1_000_000 + round(self.clock.monotonic()*400)
        super()._emit_body_state()


class TimedDemoTests(unittest.TestCase):
    def assert_body_timer(self, run):
        clock = FakeClock()
        controller = Latest400HzController(clock)
        self.addCleanup(controller.close)
        run(controller, clock)
        commands = [m for t, m in controller.transport.published if t == BODY_CMD_TOPIC]
        self.assertGreaterEqual(len(commands), 4)
        times = controller.command_times
        for dt in (b-a for a, b in zip(times, times[1:])):
            self.assertGreaterEqual(dt+1e-12, external_control.PERIOD_S)
        self.assertAlmostEqual(times[3]-times[0], 3*external_control.PERIOD_S)
        self.assertEqual([b.input_state_id-a.input_state_id
                          for a, b in zip(commands[:3], commands[1:4])], [8]*3)
        self.assertEqual(controller.calls.count('exit_external'), 1)

    def test_external_round_trip_keeps_timer_with_400hz_state(self):
        self.assert_body_timer(lambda c, clock: external_control.run(
            c, timeout=1, sleep=clock.sleep, monotonic=clock.monotonic, emit=lambda _: None))

    def test_gripper_body_hold_keeps_timer_with_400hz_state(self):
        def loop(frame):
            for _ in range(10):
                frame()
        self.assert_body_timer(lambda c, clock: gripper_external.run_external(
            c, loop, body=external_control, timeout=1, sleep=clock.sleep,
            monotonic=clock.monotonic, emit=lambda _: None))

    def test_hand_body_hold_keeps_timer_with_400hz_state(self):
        self.assert_body_timer(lambda c, clock: brainco_hand_external.run(
            c, [FakeHand()], timeout=1, sleep=clock.sleep,
            monotonic=clock.monotonic, emit=lambda _: None))

    def test_teleop_50hz_logs_actual_keys_and_keeps_timer_with_400hz_state(self):
        body, state = fixture()
        clock, stop = FakeClock(), threading.Event()
        policy = teleop_policy.Policy(METADATA, CONTRACT)
        times, commands, rows, events = [], [], [], []
        status = NS(phase='RUNNING', current_action='EXTERNAL', generation=3,
                    client_id='client', lease_id=4, reject_count=0)

        class Controller:
            def get_robot_info(self, timeout):
                return NS(info=NS(body=body), status=API.ServiceStatus.SUCCESS)

            def wait_state(self, timeout):
                state.state_id = 100+round(clock.monotonic()*400)
                state.ts_state_pub_us = 123
                state.received_realtime_ns = 124000
                state.applied_session_id = 37
                state.applied_sequence = len(commands)
                return state

            def enter_external(self, timeout):
                # The first policy sample follows a new 400 Hz frame.
                clock.sleep(1/400)
                events.append('enter')
                return status

            def publish(self, command):
                commands.append(command)
                times.append(clock.monotonic())
                return NS(session_id=37, sequence=len(commands))

            def _last_publish_timing(self):
                return dict(session_id=37, sequence=len(commands), encode_native_ns=10)

            def exit_external(self, timeout):
                events.append('cancel')
                return NS(current_action='PASSIVE')

        controller = Controller()
        controller.status = status
        teleop_policy.live(controller, API, policy, NS(infer=lambda obs: [0]*12),
                           lambda: (), stop, 1, sleep=clock.sleep,
                           monotonic=clock.monotonic, duration=.19,
                           recorder=NS(cycle=rows.append))
        self.assertEqual(len(commands), 10)
        self.assertEqual(events, ['enter', 'cancel'])
        for a, b in zip(times, times[1:]):
            self.assertAlmostEqual(b-a, .02)
        self.assertEqual([b.input_state_id-a.input_state_id
                          for a, b in zip(commands, commands[1:])], [8]*9)
        for i, row in enumerate(rows, 1):
            self.assertEqual((row['session_id'], row['sequence']), (37, i))
            self.assertEqual(row['applied_sequence_lag'], 1)
            self.assertEqual(row['publish_timing']['sequence'], i)
            self.assertEqual(row['publish_result'], 'returned')
            self.assertEqual(row['S0'], 123000)
            self.assertEqual(row['S1'], 124000)
            self.assertTrue(all(isinstance(row[f'S{n}'], int) for n in range(2, 7)))
            if i > 1:
                self.assertEqual(row['control_interval_ns'], 20_000_000)


if __name__ == '__main__':
    unittest.main()
