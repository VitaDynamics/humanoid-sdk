"""Terminal rendering consumes original snapshots without controlling the robot."""
from dataclasses import replace
import io
import unittest

from examples.end_effector import end_effector_external as demo
from examples.end_effector.live_table import LiveTable
from locomotion_aorta.end_effector_types import CommandKey, Field
from test_end_effector_codec import decode, info_object, state_object
from test_end_effector_external import Endpoint
from test_examples import FakeClock, RecordingController


class Terminal(io.StringIO):
    def isatty(self):
        return True


class EndEffectorDisplayTest(unittest.TestCase):
    def snapshot(self):
        info = decode('info', info_object(side=1, n=6))
        state = decode('state', state_object(side=1, n=6))
        state = replace(state, valid_fields=(Field.POS | Field.VEL | Field.CURRENT,) * 6,
                        vel=(0.1,) * 6, current=(0.2,) * 6, error_code=(9999,) * 6)
        return info, state

    def test_original_validity_and_intent_stay_distinct(self):
        stream = Terminal()
        info, state = self.snapshot()
        state = replace(state, output_allowed=False, inhibit_reason=4, status_valid_mask=11)
        target = demo.position_target(info, (0.02,) * 6, unit='m')
        with LiveTable(stream=stream) as display:
            display.observe_state('left', info, state)
            display.submitted('left', target, CommandKey(12, 7))
        text = stream.getvalue()
        self.assertIn('output no | inhibit ? | cmd 7', text)
        self.assertIn('0.0200', text)
        self.assertIn('0.2000', text)
        self.assertNotIn('9999', text)  # Invalid error bytes are not diagnoses.
        self.assertIn('intent, not arrival', text)
        self.assertTrue(text.endswith('\x1b[?25h'))

    def test_terminal_refresh_does_not_follow_command_frequency(self):
        clock, stream = FakeClock(), Terminal()
        with LiveTable(stream=stream, monotonic=clock.monotonic, refresh_hz=5) as display:
            for sequence in range(100):
                display(f'EE leg=low side=left submitted=sequence={sequence}')
                display('BODY_HOLD observation_only=true max_abs_q_error_rad=0.075514')
                clock.sleep(0.001)
        self.assertLess(stream.getvalue().count('End effectors |'), 5)
        self.assertNotIn('submitted=sequence=', stream.getvalue())
        self.assertIn('0.075514 rad', stream.getvalue())

    def test_redirected_output_is_plain_and_phase_bounded(self):
        stream, clock = io.StringIO(), FakeClock()
        with LiveTable(stream=stream, monotonic=clock.monotonic) as display:
            for _ in range(100):
                display('EE leg=low side=left submitted=CommandKey(...)')
                clock.sleep(0.02)
        self.assertNotIn('\x1b', stream.getvalue())
        self.assertLessEqual(stream.getvalue().count('End effectors |'), 3)

    def run_motion(self, *, table, fail=False):
        clock, stream = FakeClock(), Terminal()
        controller = RecordingController(clock)
        self.addCleanup(controller.close)
        endpoint = Endpoint(controller, hand=True)
        if fail:
            def revoke(_command):
                endpoint._state = replace(endpoint._state, output_allowed=False,
                                          inhibit_reason=4, status_valid_mask=11)
            endpoint.after_publish = revoke
        target = demo.position_target(endpoint.info, (0.2,) * 6, unit='rad')
        options = demo._MotionOptions(timeout=0.3, duration=0.08)
        def execute(emit):
            demo.run(controller, [endpoint], {'right': target}, options,
                     sleep=clock.sleep, monotonic=clock.monotonic, emit=emit)
        if table:
            with LiveTable(stream=stream, monotonic=clock.monotonic) as display:
                if fail:
                    with self.assertRaisesRegex(RuntimeError, 'side=right'):
                        execute(display)
                else:
                    execute(display)
        else:
            execute(lambda _message: None)
        return controller, endpoint, stream

    def test_display_keeps_original_command_schedule_and_single_cancel(self):
        original, first, _ = self.run_motion(table=False)
        displayed, second, _ = self.run_motion(table=True)
        self.assertEqual(original.command_times, displayed.command_times)
        self.assertEqual(first.times, second.times)
        self.assertEqual(first.commands, second.commands)
        self.assertEqual(displayed.calls.count('exit_external'), 1)

    def test_permission_failure_still_stops_after_first_target(self):
        controller, endpoint, stream = self.run_motion(table=True, fail=True)
        self.assertEqual(len(endpoint.commands), 1)
        self.assertEqual(controller.calls.count('exit_external'), 1)
        self.assertTrue(stream.getvalue().endswith('\x1b[?25h'))

    def test_exception_restores_cursor_and_keeps_full_diagnostic(self):
        stream = Terminal()
        diagnostic = 'permission revoked: ' + 'original-state-' * 20
        with self.assertRaisesRegex(RuntimeError, 'permission revoked'):
            with LiveTable(stream=stream):
                raise RuntimeError(diagnostic)
        self.assertIn('\x1b[?25h', stream.getvalue())
        self.assertIn(diagnostic, stream.getvalue())

    def test_invalid_display_rate_is_rejected(self):
        for rate in (0, -1, float('nan'), float('inf')):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                LiveTable(stream=io.StringIO(), refresh_hz=rate)


if __name__ == '__main__':
    unittest.main()
