"""Numerical parity and bounded control behavior; never opens a robot session."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest('install .[wbc-tracking-demo] to run tracking math tests')

from locomotion_aorta import ControlStatus, ExternalPhase, BodyImuSample, BodyImuState
from locomotion_aorta.wbc_tracking_policy import (
    CONTRACT_KEY, Contract, Motion, WbcTrackingPolicy, yaw_quaternion,
)

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('tracking_example', ROOT/'examples/wbc_tracking_mode/wbc_tracking_mode.py')
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


def policy():
    raw = (ROOT/'tests/fixtures/actor_g1_contract.json').read_text()
    contract = Contract.from_metadata({CONTRACT_KEY: raw})
    return WbcTrackingPolicy(contract, json.loads((ROOT/'examples/wbc_tracking_mode/wbc_tracking_asample.json').read_text()))


class TrackingMathTest(unittest.TestCase):
    def setUp(self):
        self.p = policy()

    def test_two_ticks_match_cpp_observation_golden(self):
        # Same independently generated fixture used by C++ GrootObservationGolden.
        # That fixture samples every 20 ms; this model's metadata asks for 100 ms.
        self.p.contract = replace(self.p.contract, future_dt=.02)
        data = json.loads((ROOT/'tests/fixtures/groot_observation_golden.json').read_text())
        motion = Motion(np.reshape(data['joint_pos'], (-1, 28)),
                        np.reshape(data['base_pos'], (-1, 3)),
                        np.reshape(data['base_quat'], (-1, 4)))
        for tick in data['ticks']:
            self.p.last_action = np.array(tick['last_action'], np.float32)
            actual = self.p.observe(motion, tick['ref_t'], tick['q'], tick['qd'],
                                    tick['robot_quat'], tick['base_ang_vel_body'],
                                    tick['yaw_offset_rad'])
            np.testing.assert_allclose(actual, tick['obs'], rtol=0, atol=1e-7)

    def test_metadata_controls_future_stride(self):
        c = self.p.contract
        self.assertEqual((c.future_frames, c.future_dt, c.history_length, c.input_dim),
                         (10, .1, 10, 1520))
        q = np.repeat(np.arange(60)[:, None], 28, axis=1)
        motion = Motion(q, np.zeros((60, 3)), np.tile([1, 0, 0, 0], (60, 1)))
        obs = self.p.observe(motion, .02, c.default, np.zeros(28), [1, 0, 0, 0], [0, 0, 0])
        np.testing.assert_equal(obs[:280].reshape(10, 28)[:, 0], np.arange(1, 47, 5))

    def test_target_mapping_limits_and_ankle_feedforward(self):
        p, c = self.p, self.p.contract
        action = np.linspace(-30, 30, 28)
        command, decoded = p.decode(action)
        clipped = np.clip(action, -20, 20).astype(np.float32)
        raw = c.default+c.scale*clipped
        safe = np.clip(raw, p.lower, p.upper)
        ankle_names = {f'{side}_ankle_{axis}_joint' for side in ('left', 'right') for axis in ('pitch', 'roll')}
        self.assertEqual(len(command.joints), 31)
        for j, name in enumerate(c.names):
            motor = command.joints[p.order.index(name)]
            self.assertEqual(getattr(motor, "mode", None), None)
            self.assertEqual(motor.q, float(np.float32(safe[j])))
            self.assertEqual(motor.kp, float(np.float32(c.kp[j])))
            self.assertEqual(motor.kd, float(np.float32(c.kd[j])))
            expected_tau = c.kp[j]*(raw[j]-safe[j]) if name in ankle_names else 0
            self.assertAlmostEqual(motor.tau, float(np.float32(expected_tau)))
        self.assertTrue(any(decoded['clamped']))
        self.assertTrue(any(x != 0 for x in decoded['tau_ff']))
        np.testing.assert_equal(p.last_action, clipped)
        for name in ('neck_pitch_joint', 'neck_yaw_joint'):
            motor = command.joints[p.order.index(name)]
            self.assertEqual((motor.q, motor.kp, motor.kd), (0., 20., 1.))
        for name in p.order:
            if name not in c.names and not name.startswith('neck_'):
                motor = command.joints[p.order.index(name)]
                self.assertEqual((motor.q, motor.kp, motor.kd, motor.tau), (0., 0., 0., 0.))

    def test_history_feedback_is_clipped_network_action(self):
        self.p.decode(np.full(28, 25.))
        state = example.synthetic_state(self.p)
        session = example.ReferenceSession(self.p, state)
        obs = session.observation(state)
        np.testing.assert_equal(obs[620+30+280+280:620+30+280+280+280], 20.)
        self.p.reset()
        self.assertFalse(self.p.history)
        np.testing.assert_equal(self.p.last_action, 0.)

    def test_hip_selected_by_location_not_array_order(self):
        state = example.synthetic_state(self.p)
        trunk = BodyImuSample(1, BodyImuState((0.,)*4, (0.,)*3, (0.,)*3, (0.,)*3), 6)
        good = replace(state, imu_states=(trunk, state.imu_states[0]))
        q, _, quat, _ = self.p.state(good)
        np.testing.assert_equal(q, self.p.contract.default)
        np.testing.assert_equal(quat, [1., 0., 0., 0.])
        for bad in (replace(state, imu_states=(trunk,)),
                    replace(state, attitude_valid=False),
                    replace(state, joints=state.joints[:30]),
                    replace(state, imu_states=(replace(state.imu_states[0], status=6),)),
                    replace(state, imu_states=(state.imu_states[0],)*2),
                    replace(state, imu_states=(replace(state.imu_states[0], state=replace(state.imu_states[0].state, quaternion=(0.,)*4)),)),
                    replace(state, imu_states=(replace(state.imu_states[0], state=replace(state.imu_states[0].state, quaternion=(.7, .7, 0., 0.))),))):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.p.state(bad)

    def test_nonfinite_action_and_state_fail(self):
        for bad in (np.full(28, np.nan), np.full(28, np.inf), np.zeros(31)):
            with self.assertRaises(ValueError):
                self.p.decode(bad)
        state = example.synthetic_state(self.p)
        motors = (replace(state.joints[0], q=float('nan')),)+state.joints[1:]
        with self.assertRaises(ValueError):
            self.p.state(replace(state, joints=motors))

    def test_unknown_model_layout_fails(self):
        with self.assertRaises(ValueError):
            Contract.from_metadata({})
        original = json.loads((ROOT/'tests/fixtures/actor_g1_contract.json').read_text())
        for key, value in (('action_dim', 31), ('input_dim', 100),
                           ('actor_history_order', 'frame_major')):
            with self.subTest(key=key), self.assertRaises(ValueError):
                Contract.from_metadata({CONTRACT_KEY: json.dumps(dict(original, **{key:value}))})

    def test_reference_terminal_velocity_and_rounding_match_cpp(self):
        q = np.repeat(np.array([0., 1., 4., 9.])[:, None], 28, axis=1)
        motion = Motion(q, np.zeros((4, 3)), np.tile([1, 0, 0, 0], (4, 1)))
        np.testing.assert_equal(motion.velocities(np.arange(4))[:, 0], [50., 150., 250., 150.])
        np.testing.assert_equal(motion.indices(.01, 4, .02), [0, 1, 2, 3])
        np.testing.assert_equal(motion.indices(.03, 4, .02), [2, 3, 3, 3])

    def test_npz_names_resampling_and_lead_in(self):
        names = self.p.contract.names
        q = np.stack([self.p.contract.default+i*.01 for i in range(3)])
        quat = np.array([yaw_quaternion(.3+i*.1) for i in range(3)])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'motion.npz'
            np.savez(path, joint_names=list(reversed(names)), joint_pos=q[:, ::-1],
                     base_pos_w=np.zeros((3, 3)), base_quat_w=quat, framerate=25.)
            motion = Motion.from_npz(path, names)
        self.assertEqual(len(motion.q), 5)
        np.testing.assert_allclose(motion.q[1], (q[0]+q[1])/2, rtol=0, atol=1e-12)
        lead = motion.lead_in(q[0]-.1, [1, 0, 0, 0], self.p.profile['lead_in'])
        self.assertEqual(len(lead.q), 15+5)
        np.testing.assert_allclose(lead.q[0], q[0]-.1)
        np.testing.assert_allclose(lead.q[15:], motion.q)
        np.testing.assert_allclose(lead.quat[15], [1, 0, 0, 0], atol=1e-12)


class MemoryRecorder:
    def __init__(self):
        self.rows = []

    def write(self, row):
        self.rows.append(row)


class FakeModel:
    def __init__(self):
        self.calls = 0
        self.after = lambda: None

    def infer(self, obs):
        self.calls += 1
        self.after()
        return np.zeros(28, np.float32)


class FakeClock:
    def __init__(self):
        self.now = 100.

    def monotonic(self):
        return self.now

    def sleep(self, dt):
        self.now += dt


class FakeController:
    def __init__(self, p, clock):
        self.clock, self.state = clock, example.synthetic_state(p)
        self.status = ControlStatus(1, 'EXTERNAL', ExternalPhase.RUNNING,
                                    'tracking-test', 'lease', generation=9)
        self.commands, self.exits, self.enters = [], 0, 0
        self.advance = True

    def wait_body_state(self, timeout):
        if self.advance:
            self.state = replace(self.state, state_id=self.state.state_id+1,
                                 ts_state_real_us=self.state.ts_state_real_us+2000,
                                 received_monotonic_ns=round(self.clock.now*1e9))
        return self.state
    wait_state = wait_body_state

    def enter_external(self, timeout):
        self.enters += 1
        return self.status

    def publish(self, command):
        command = replace(command, session_id=2**63+9, sequence=2**63+1001+len(self.commands))
        self.commands.append(command)
        return command

    def exit_external(self, timeout):
        self.exits += 1
        return replace(self.status, current_action='PASSIVE', phase=ExternalPhase.CANCELLED)


class TrackingLoopTest(unittest.TestCase):
    def setUp(self):
        self.p, self.clock, self.model, self.rec = policy(), FakeClock(), FakeModel(), MemoryRecorder()
        self.controller = FakeController(self.p, self.clock)
        self.args = SimpleNamespace(motion=None, timeout=1., max_state_age_ms=50.,
                                    max_compute_ms=50., stand_seconds=3., duration=.041)

    def run_live(self):
        return example.live(self.model, self.p, self.args, self.rec, self.controller,
                            source=self.controller, monotonic=self.clock.monotonic,
                            sleep=self.clock.sleep)

    def test_bounded_50hz_loop_uses_new_state_and_exits(self):
        result = self.run_live()
        self.assertEqual((result['steps'], self.model.calls), (3, 4))
        self.assertEqual((self.controller.enters, self.controller.exits), (1, 1))
        commands = self.controller.commands
        rows = [r for r in self.rec.rows if r['kind'] == 'step' and r['published']]
        self.assertEqual([(r['session_id'], r['sequence']) for r in rows],
                         [(c.session_id, c.sequence) for c in commands])
        self.assertTrue(all(r['session_id'] > 2**63 and r['sequence'] > 2**63 for r in rows))
        self.assertEqual(len(set(c.input_state_id for c in commands)), 3)
        self.assertTrue(all(c.input_state_ts_real_us > 0 for c in commands))
        for i, row in enumerate(rows):
            self.assertTrue(all(isinstance(row[f'S{n}'], int) for n in range(2, 7)))
            self.assertEqual(row['publish_result'], 'returned')
            self.assertEqual(row['control_interval_ns'], 'NOT_MEASURED' if i == 0 else 20_000_000)
            self.assertEqual(row['applied_sequence_lag'], 'NOT_MEASURED')
        self.assertEqual(self.rec.rows[-1]['current_action'], 'PASSIVE')

    def test_policy_is_50hz_with_400hz_latest_state_source(self):
        self.args.duration=.19
        source_ids=[];times=[];start=self.clock.now
        original_state=self.controller.state
        def latest(timeout):
            sample_index=round((self.clock.now-start)*400)
            source_ids.append(sample_index)
            return replace(original_state,state_id=sample_index,
                           ts_state_real_us=original_state.ts_state_real_us+sample_index*2500,
                           received_monotonic_ns=round(self.clock.now*1e9))
        self.controller.wait_state=latest
        original_publish=self.controller.publish
        def publish(command):
            times.append(self.clock.now)
            return original_publish(command)
        self.controller.publish=publish
        result=self.run_live()
        self.assertEqual(result['steps'],10)
        np.testing.assert_allclose(np.diff(times),[.02]*9,rtol=0,atol=1e-12)
        np.testing.assert_array_equal(np.diff([c.input_state_id for c in self.controller.commands]),[8]*9)
        self.assertEqual(self.model.calls,11)  # one warmup; ten 50Hz policy evaluations.

    def test_publish_timing_is_joined_to_the_actual_returned_key(self):
        self.controller._last_publish_timing = lambda: dict(
            session_id=self.controller.commands[-1].session_id,
            sequence=self.controller.commands[-1].sequence, encode_native_ns=123)
        self.run_live()
        for row in [x for x in self.rec.rows if x.get('published')]:
            self.assertEqual((row['session_id'], row['sequence']),
                             (row['publish_timing']['session_id'], row['publish_timing']['sequence']))
            self.assertEqual(row['publish_timing']['encode_native_ns'],123)

    def test_publish_failure_survives_a_diagnostic_write_failure(self):
        self.controller.publish = lambda _: (_ for _ in ()).throw(RuntimeError('send failed'))
        original = self.rec.write
        def write(row):
            if row.get('published') is False:raise OSError('log full')
            original(row)
        self.rec.write = write
        with self.assertRaisesRegex(RuntimeError,'send failed'):self.run_live()
        self.assertEqual(self.controller.exits,1)

    def test_stale_cache_and_nonadvancing_state_stop(self):
        state = self.controller.wait_body_state(.1)
        self.controller.advance = False
        self.clock.now += .1
        with self.assertRaises(TimeoutError):
            example.wait_fresh(self.controller, self.p, previous_ns=0, timeout=.05,
                               max_age=.05, stop=threading.Event(),
                               monotonic=self.clock.monotonic, sleep=self.clock.sleep)
        self.clock.now = state.received_monotonic_ns/1e9
        with self.assertRaises(TimeoutError):
            example.wait_fresh(self.controller, self.p, previous_ns=state.stamp_ns, timeout=.05,
                               max_age=.05, stop=threading.Event(),
                               monotonic=self.clock.monotonic, sleep=self.clock.sleep)

    def test_inference_failure_cancels_without_publication(self):
        def fail_after_warmup():
            if self.model.calls > 1:
                raise RuntimeError('inference failed')
        self.model.after = fail_after_warmup
        with self.assertRaisesRegex(RuntimeError, 'inference failed'):
            self.run_live()
        self.assertFalse(self.controller.commands)
        self.assertEqual(self.controller.exits, 1)

    def test_lost_ownership_during_inference_cancels_without_publication(self):
        def lose():
            if self.model.calls > 1:
                self.controller.status = replace(self.controller.status, generation=10)
        self.model.after = lose
        with self.assertRaisesRegex(RuntimeError, 'changed during inference'):
            self.run_live()
        self.assertFalse(self.controller.commands)
        self.assertEqual(self.controller.exits, 1)

    def test_50ms_compute_deadline_cancels_without_publication(self):
        with patch.object(example.time, 'monotonic_ns', side_effect=range(0, 10**9, 30_000_000)):
            with self.assertRaisesRegex(TimeoutError, 'max-compute'):
                self.run_live()
        self.assertFalse(self.controller.commands)
        self.assertEqual(self.controller.exits, 1)
        self.assertFalse(self.rec.rows[-2]['published'])

    def test_stop_before_entry_does_not_request_control(self):
        stop = threading.Event()
        stop.set()
        example.live(self.model, self.p, self.args, self.rec, self.controller,
                     source=self.controller, stop=stop)
        self.assertEqual((self.controller.enters, self.controller.exits), (0, 0))

    def test_live_requires_an_explicit_state_source_before_entry(self):
        with self.assertRaises(TypeError):
            example.live(self.model, self.p, self.args, self.rec, self.controller)
        self.assertEqual((self.controller.enters, self.controller.exits), (0, 0))

    def test_unconfirmed_exit_is_reported_as_failure(self):
        self.controller.exit_external = lambda timeout: replace(
            self.controller.status, current_action='EXTERNAL')
        with patch.object(example.sys, 'stderr'), self.assertRaisesRegex(RuntimeError, 'PASSIVE'):
            self.run_live()
        self.assertEqual(self.rec.rows[-1]['kind'], 'exit_failure')

    def test_exit_error_does_not_hide_original_failure(self):
        def fail_after_warmup():
            if self.model.calls > 1:
                raise RuntimeError('inference failed')
        self.model.after = fail_after_warmup
        self.controller.exit_external = lambda timeout: (_ for _ in ()).throw(TimeoutError('exit timeout'))
        with patch.object(example.sys, 'stderr'), self.assertRaisesRegex(RuntimeError, 'inference failed'):
            self.run_live()
        self.assertEqual(self.rec.rows[-1]['kind'], 'exit_failure')

    def test_recorder_never_overwrites_existing_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'trial.jsonl'
            path.write_text('previous')
            with self.assertRaises(FileExistsError):
                example.Recorder(path, {})
            self.assertEqual(path.read_text(), 'previous')


if __name__ == '__main__':
    unittest.main()
