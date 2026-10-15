"""Per-command parity with the split demo at c231c4f; synthetic, never HIL.

Each digest covers the complete dataclass payload, including all body joints,
keys and timestamps. UUID and client-clock sources are fixed in this test only.
The rows preserve publication order and simulated time across body and grippers.
"""
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

from examples import gripper_external as demo
from locomotion_aorta import BODY_CMD_TOPIC
from test_examples import FakeClock, RecordingController
from test_fixed_stand_pose import body_info
from body_fixtures import robot_info_reply
from test_gripper_external import FakeGripper


BASE = 'c231c4fb2a476af3051f363ced13d9880f553271'
CASES = {
    'single_target': dict(zero=False, sine=False, back=False),
    'single_round_trip': dict(zero=False, sine=False, back=True),
    'zero_single_round_trip': dict(zero=True, sine=False, back=True),
    'dual_sine_measured_hold': dict(zero=False, sine=True),
    'dual_sine_zero_hold': dict(zero=True, sine=True),
    'single_rejected': dict(zero=False, sine=False, back=False, reject=True),
}


def command_trace(case):
    options = CASES[case]
    clock, rows, payloads = FakeClock(), [], []
    c = RecordingController(clock)
    previous = c.transport.on_publish
    def record(topic, command):
        data = asdict(command)
        payloads.append(data)
        digest = hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'),
                                           allow_nan=False).encode()).hexdigest()
        rows.append([topic, clock.monotonic(), digest])
    def on_publish(topic, command):
        record(topic, command)
        previous(topic, command)
    c.transport.on_publish = on_publish
    clients = []
    for side, wire, q in [('left', 1, .02), ('right', 2, .07)]:
        g = FakeGripper(reject=options.get('reject', False))
        g.side, g.session_id = side, 77 + wire
        g.info = replace(g.info, side=wire)
        g._state = replace(g._state, side=wire, sample=replace(g._state.sample, q=q))
        publish = g.publish_target
        def gripper_publish(*a, g=g, publish=publish, **kw):
            cmd = replace(publish(*a, **kw), side=g.info.side)
            g.commands[-1] = cmd
            record('/sdk/foot_humanoid/gripper/' + g.side + '/cmd', cmd)
            return cmd
        g.publish_target = gripper_publish
        clients.append(g)
    response = robot_info_reply()
    c.get_robot_info = lambda timeout: replace(response, info=replace(
        response.info, body=body_info(), end_effectors=tuple(SimpleNamespace(
            configured=True, side=g.info.side, gripper=g.info) for g in clients)))
    error = None
    try:
        with patch('locomotion_aorta.controller.time.monotonic_ns',
                   side_effect=lambda: int(clock.monotonic()*1e9)), \
             patch('locomotion_aorta.controller.uuid.uuid4', return_value=uuid.UUID(int=1)):
            params = dict(timeout=1, sleep=clock.sleep, monotonic=clock.monotonic,
                          emit=lambda _: None, body_pose='zero' if options['zero'] else 'measured')
            try:
                if options['sine']:
                    demo.run_sine(c, clients, duration=.4, period=.2, **params)
                else:
                    demo.run(c, clients[1], q=.075, duration=.08,
                             return_to_start=options['back'], **params)
            except RuntimeError as exc:
                error = str(exc)
    finally:
        c.close()
    return {'rows': rows, 'error': error}, payloads


class ConsolidationParityTest(unittest.TestCase):
    def test_every_command_and_time_matches_the_preconsolidation_demo(self):
        fixture = json.loads((Path(__file__).parent/'fixtures/gripper-demo-c231c4f.json').read_text())
        self.assertEqual(fixture['source_head'], BASE)
        self.assertEqual(set(fixture['cases']), set(CASES))
        for case in CASES:
            with self.subTest(case=case):
                actual, payloads = command_trace(case)
                expected = fixture['cases'][case]
                self.assertEqual(actual['error'], expected['error'])
                self.assertEqual(len(actual['rows']), len(expected['rows']))
                for index, (got, want) in enumerate(zip(actual['rows'], expected['rows'])):
                    self.assertEqual(got, want, (case, index, payloads[index]))
                body_rows = [r for r in actual['rows'] if r[0] == BODY_CMD_TOPIC]
                self.assertTrue(body_rows)
