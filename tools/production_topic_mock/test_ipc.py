"""Real compiled Mock + SDK IPC on private loopback peers; no robot endpoints."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest


def client(output):
    from locomotion_aorta import (BodyCmd, BodyJointCmd, PullController,
                                  PullStateSource)
    group = os.environ['AORTA_GROUP']
    with PullController(group=group) as controller:
        source = PullStateSource(group=group)
        try:
            info = controller.get_robot_info(5)
            if info.info is None or info.info.body.slot_count != 31:
                raise AssertionError('31-slot RobotInfo missing')
            state = source.wait_state(5)
            controller.enter_external(5)
            try:
                for _ in range(3):
                    state = source.wait_state(0)
                    sent = controller.publish(BodyCmd(
                        joints=tuple(BodyJointCmd() if i == 14 else
                                     BodyJointCmd(q=float(state.q[i]), kp=1, kd=.5)
                                     for i in range(31)),
                        input_state_id=state.state_id,
                        input_state_ts_real_us=state.ts_state_real_us))
                    time.sleep(.02)
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    dto = controller.wait_body_state(0)
                    if (dto.applied_session_id, dto.applied_sequence) == (sent.session_id, sent.sequence):
                        break
                    time.sleep(.005)
                else:
                    raise AssertionError('last command was not applied and echoed')
            finally:
                status = controller.exit_external(5)
            if status.current_action != 'PASSIVE':
                raise AssertionError('CANCEL did not return PASSIVE')
        finally:
            source.close()
    (output / 'client-result.json').write_text(json.dumps(dict(commands=3, nodes=2, passive=True)))


class MockIpcTest(unittest.TestCase):
    def test_two_nodes_prepare_commit_publish_and_cancel(self):
        from . import run
        build = Path(os.environ['MOCK_BUILD_DIR']).resolve()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / 'run'
            args = run.parser().parse_args([
                '--mode', 'production-topics', '--build-dir', str(build),
                '--output', str(output), '--seconds', '5', '--allow-client',
                '--client-command', sys.executable, str(Path(__file__).resolve()),
                '--client', str(output)])
            rc = run.run(args)
            logs = '\n'.join(p.read_text() for p in output.glob('*.log'))
            self.assertEqual(rc, 0, logs)
            result = json.loads((output / 'mock-result.json').read_text())
            self.assertEqual(result['endpoint_rc'], 0)
            self.assertEqual(result['parse_errors'], 0)
            self.assertEqual(result['accepted'], 3)
            self.assertGreaterEqual(result['applied'], 1)
            self.assertEqual(result['control_phase_at_stop'], 0)
            self.assertEqual(json.loads((output / 'client-result.json').read_text()),
                             dict(commands=3, nodes=2, passive=True))
            self.assertFalse(list(output.glob('*.csv')))


if __name__ == '__main__':
    if sys.argv[1:2] == ['--client']:
        client(Path(sys.argv[2]))
    else:
        unittest.main()
