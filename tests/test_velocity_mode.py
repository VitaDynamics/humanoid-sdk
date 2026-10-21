import json
import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

missing = [name for name in ("numpy", "yaml") if importlib.util.find_spec(name) is None]
if missing:
    raise unittest.SkipTest(f"optional velocity mode dependencies unavailable: {', '.join(missing)}")
import numpy as np
from examples.velocity_mode import velocity_mode as example
from examples.velocity_mode import velocity_mode_latency as latency
from examples.velocity_mode.velocity_mode_latency import CycleLog, NOT_MEASURED
from examples.velocity_mode.velocity_mode_policy import VelocityModePolicy


def frame_state(state_id):
    from locomotion_aorta import BodyState, BodyJointState
    return BodyState(state_id, (BodyJointState(),)*31, ts_state_real_us=1000)


def policy_fixture():
    policy = VelocityModePolicy.__new__(VelocityModePolicy)
    policy.names = ["left_hip", "right_hip"]
    policy.order = [f"slot_{i}" for i in range(31)]
    policy.order[0], policy.order[6] = policy.names
    policy.policy_to_body = [0, 6]
    policy.config = {
        "command": dict(vx_range=[-0.6, 0.9], vy_range=[0.0, 0.0], wz_range=[-0.8, 0.8],
                        joy_axis_vx=1, joy_axis_vy=0, joy_axis_wz=3, deadband=0.05,
                        stand_enabled=True, stand_vxy_deadband=0.03, stand_wz_deadband=0.05),
        "fixed_joints": {"slot_29": {"q": 0, "kp": 20, "kd": 1},
                         "slot_30": {"q": 0, "kp": 20, "kd": 1}},
    }
    policy.driven_slots = {0, 6, 29, 30}
    policy.default_q = np.array([0.1, -0.2], dtype=np.float64)
    policy.kp, policy.kd = np.array([50., 60.]), np.array([2., 3.])
    policy.lower, policy.upper = np.array([-1., -1.]), np.array([1., 1.])
    policy.action_scale = np.array([0.5, 0.25])
    policy.metadata = dict(clip_actions="2", clip_observations="100",
                           obs_scale_base_ang_vel="0.2", obs_scale_joint_vel="0.05")
    policy.terms = ["base_ang_vel", "projected_gravity", "command", "joint_pos", "joint_vel", "actions"]
    policy.history_len, policy.frame_dim = 5, 15
    policy.reset()
    return policy


class PolicyTest(unittest.TestCase):
    def setUp(self):
        self.policy = policy_fixture()

    def test_asymmetric_joy_endpoints_and_no_lateral_motion(self):
        np.testing.assert_array_equal(self.policy.command((1., 1., 0., -1.)), [0.9, 0., -0.8])
        np.testing.assert_array_equal(self.policy.command((-1., -1., 0., 1.)), [-0.6, 0., 0.8])

    def test_deadband_boundary_and_missing_input_is_zero(self):
        np.testing.assert_array_equal(self.policy.command(()), [0., 0., 0.])
        np.testing.assert_array_equal(self.policy.command((0., .049, 0., 0.)), [0., 0., 0.])
        self.assertAlmostEqual(self.policy.command((0., .05, 0., 0.))[0], .045)

    def test_negative_joy_keeps_the_existing_profile_mapping(self):
        np.testing.assert_array_equal(self.policy.command((0., -1., 0., 0.)), [-.6, 0., 0.])

    def observe(self, gyro=(0., 0., 0.)):
        q, dq = [0.] * 31, [0.] * 31
        q[0], q[6] = .3, .4
        q[14], q[15] = 10000., -10000.
        return self.policy.observe(q, dq, gyro, (1., 0., 0., 0.), np.zeros(3))

    def test_state_slots_permute_by_name_and_reserved_slots_are_excluded(self):
        obs = self.observe().reshape(5, 15)
        np.testing.assert_allclose(obs[0, 9:11], [.2, .6], atol=1e-7)
        np.testing.assert_array_equal(obs[0, 3:6], [0., 0., -1.])
        self.assertLess(abs(obs).max(), 100)

    def test_initial_backfill_shift_and_reset(self):
        first = self.observe().reshape(5, 15)
        np.testing.assert_array_equal(first, np.tile(first[0], (5, 1)))
        second = self.observe((1., 0., 0.)).reshape(5, 15)
        np.testing.assert_array_equal(second[:4], first[:4])
        self.assertAlmostEqual(float(second[-1, 0]), .2)
        self.policy.reset()
        reset = self.observe((2., 0., 0.)).reshape(5, 15)
        np.testing.assert_array_equal(reset, np.tile(reset[0], (5, 1)))

    def test_action_feedback_is_clipped_before_position_decode(self):
        motors = self.policy.decode(np.array([10., -10.], dtype=np.float32))
        np.testing.assert_array_equal(self.policy.last_action, [2., -2.])
        self.assertEqual(motors[0]["q"], 1.0)
        self.assertAlmostEqual(motors[6]["q"], -.7, places=6)
        self.assertEqual(motors[0]["kp"], 50.)
        self.assertEqual(motors[6]["kd"], 3.)

    def test_public_31_hole_and_fixed_neck(self):
        motors = self.policy.decode(np.zeros(2, dtype=np.float32))
        self.assertEqual(len(motors), 31)
        for slot in (14,):
            self.assertEqual(motors[slot], dict(q=0., dq=0., tau=0., kp=0., kd=0.))
        for slot in (29, 30):
            self.assertEqual((motors[slot]["q"], motors[slot]["kp"], motors[slot]["kd"]), (0., 20., 1.))


class LoggingTest(unittest.TestCase):
    def test_run_header_uses_the_bound_robot_roster(self):
        policy = policy_fixture()
        policy.inventory = dict(body_joint_names=policy.order,
                                policy_to_body_slot=policy.policy_to_body,
                                fixed_joints={})
        names = tuple(reversed(policy.order))
        body = SimpleNamespace(joint_names=names, roster_version=1,
                               joints=[SimpleNamespace(populated=True, is_mock=False)]*31)
        transport, directions = Mock(), Mock()
        transport.get_robot_info.return_value = SimpleNamespace(info=SimpleNamespace(body=body))
        transport.observed_status = SimpleNamespace(current_action="PASSIVE",
            phase=SimpleNamespace(value="IDLE"), generation=0)
        transport.publishers, directions.node.publishers = {}, []
        directions.statistics.return_value = {}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"run.jsonl"
            args = example.parse_args(["--shadow", "--locomotion-dir", "/unused", "--log", str(path)])
            with patch.object(example, "version", return_value="0.2.1"), \
                 patch.object(example, "parse_args", return_value=args), \
                 patch.object(example, "VelocityModePolicy", return_value=policy), \
                 patch.object(example, "PolicyTransport", return_value=transport), \
                 patch.object(example, "DirectionInput", return_value=directions), \
                 patch.object(example, "transition_duration", return_value=([], 0.)), \
                 patch.object(example, "run_cycles"):
                example.main()
            header = json.loads(path.read_text().splitlines()[0])
        self.assertEqual(header["models"][0]["body_joint_names"], list(names))
        self.assertEqual(header["models"][0]["policy_to_body_slot"], [30, 24])
        self.assertEqual(header["models"][0]["fixed_joints"], policy.config["fixed_joints"])

    def test_cycle_fields_round_trip_and_overhead_has_its_own_record(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "cycles.jsonl"
            log = CycleLog(path, {"mode": "shadow"})
            log.cycle(dict(cycle=1, S0=100, S1=101, S2=102, S3=103, S4=104, S5=105,
                           S6=NOT_MEASURED, L1=NOT_MEASURED, L2=NOT_MEASURED,
                           sequence=NOT_MEASURED))
            log.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([r["record_type"] for r in rows],
                             ["run", "cycle", "logging_overhead", "logging_summary"])
            self.assertEqual(rows[1]["S6"], NOT_MEASURED)
            self.assertGreater(rows[2]["cycle_record_write_ns"], 0)
            self.assertEqual(rows[3]["N"], 1)

    def test_logging_overhead_and_summary_use_realtime_nanoseconds(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(latency.time, "time_ns", side_effect=range(100, 300, 10)), \
             patch.object(latency.time, "monotonic", side_effect=AssertionError("monotonic used")), \
             patch.object(latency.time, "perf_counter_ns", side_effect=AssertionError("perf used")):
            path = Path(temp) / "clock.jsonl"
            log = CycleLog(path, {})
            log.cycle(dict(cycle=1))
            log.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(rows[0]["started_ns"], 100)
            self.assertEqual(rows[2]["cycle_record_write_ns"], 10)
            self.assertEqual(rows[3]["clock"], "CLOCK_REALTIME")
            self.assertEqual(rows[3]["finished_ns"], 170)
            self.assertEqual(rows[3]["ms"]["max"], 20 / 1e6)

    def test_logging_realtime_step_is_not_hidden_by_clamping(self):
        log = CycleLog.__new__(CycleLog)
        log.stream = Mock()
        with patch.object(latency.time, "time_ns", side_effect=(100, 90)):
            self.assertEqual(log.write({}), -10)

    def test_logger_refuses_to_overwrite_existing_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "cycles.jsonl"
            path.write_text("existing")
            with self.assertRaises(FileExistsError):
                CycleLog(path, {})
            self.assertEqual(path.read_text(), "existing")


class DirectionSourceTest(unittest.TestCase):
    def test_only_aorta_input_and_old_options_are_rejected(self):
        from contextlib import redirect_stderr
        import io
        for mode in ("--shadow", "--live"):
            args = [mode, "--locomotion-dir", "/unused", "--log", "/unused.jsonl"]
            self.assertFalse(hasattr(example.parse_args(args), "joy_source"))
            for source in ("aorta", "ros2"):
                with self.subTest(mode=mode, source=source), redirect_stderr(io.StringIO()), \
                     self.assertRaises(SystemExit) as error:
                    example.parse_args([*args, "--joy-source", source])
                self.assertEqual(error.exception.code, 2)

    def fixture(self):
        from test_joy import PullRing
        ring = PullRing(256)
        node = SimpleNamespace(create_subscriber_pull=Mock(return_value=ring))
        unavailable = {name: None for name in ("rclpy", "sensor_msgs", "geometry_msgs")}
        with patch.dict(sys.modules, unavailable), patch.object(example.threading, "Thread") as thread:
            directions = example.DirectionInput(SimpleNamespace(node=node))
            thread.assert_not_called()
        self.addCleanup(directions.close)
        return directions, ring

    def test_real_joy_reader_routes_axes_and_preserves_event_counts(self):
        from test_joy import payload
        directions, ring = self.fixture()
        with patch.object(example.time, "time_ns", return_value=1_000_000_000):
            ring.put(payload(0))
            directions.snapshot()
            ring.put(payload(1, axes=(0., 1., 0., -1.), buttons=(1,)),
                     payload(2, axes=(0., 1., 0., -1.), buttons=(0,)))
            axes = directions.snapshot()
        np.testing.assert_array_equal(policy_fixture().command(axes), [.9, 0., -.8])
        stats = directions.statistics()
        self.assertEqual(stats["joy_messages"], 3)
        self.assertEqual(stats["joy_pressed_counts"], {0: 1})
        self.assertEqual(stats["joy_released_counts"], {0: 1})
        self.assertEqual(stats["last_joy_ns"], 1_000_000_000)
        json.dumps(stats)

    def test_realtime_freshness_boundary_and_no_stale_fallback(self):
        from test_joy import payload
        directions, ring = self.fixture()
        received = 1_700_000_000_000_000_000
        axes = (0., 1., 0., 0.)
        with patch.object(example.time, "time_ns", return_value=received):
            ring.put(payload(0, axes=axes))
            self.assertEqual(directions.snapshot(), axes)
        with patch.object(example.time, "time_ns", return_value=received+1_000_000_000):
            self.assertEqual(directions.snapshot(), axes)
        for now in (received+1_000_000_001, received-1):
            with self.subTest(now=now), patch.object(example.time, "time_ns", return_value=now):
                self.assertEqual(directions.snapshot(), ())
        self.assertEqual(directions.statistics()["last_joy_ns"], received)

    def test_reader_errors_propagate_and_close_owns_only_subscription(self):
        directions, ring = self.fixture()
        ring.queue.append(RuntimeError("disconnected"))
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            directions.snapshot()
        directions.close()
        ring.close.assert_called_once_with()


class TransportTest(unittest.TestCase):
    def test_shadow_never_constructs_any_aorta_publisher(self):
        calls = []

        class Node:
            def __init__(self, *args):
                pass

            def create_publisher_typed(self, *args, **kwargs):
                calls.append(args)
                raise AssertionError("shadow constructed a publisher")

            def close(self):
                pass

        Node.create_subscriber_pull = lambda self, *a, **k: SimpleNamespace(close=lambda: None)
        qos = SimpleNamespace(sensor_data=lambda: 'sensor', state_update=lambda: 'state')
        bindings = SimpleNamespace(aorta=SimpleNamespace(Node=Node, QoS=qos))
        with patch.dict(example.os.environ, {}, clear=True), \
             patch("locomotion_aorta.transport._load_aorta_bindings", return_value=bindings):
            transport = example.PolicyTransport(group="default", shadow=True)
            self.assertEqual(transport.publishers, {})
            self.assertEqual(example.os.environ["AORTA_LOG_TOPIC_ENABLED"], "false")
            transport.close()
        self.assertEqual(calls, [])

    def test_public_command_has_no_legacy_id(self):
        from locomotion_aorta import BodyCmd, BodyJointCmd
        command = BodyCmd((BodyJointCmd(),) * 31, session_id=2**63, sequence=2**64-1)
        self.assertEqual((command.session_id, command.sequence), (2**63, 2**64-1))
        self.assertFalse(hasattr(command, 'cmd_id'))

    def test_preflight_refuses_existing_external_session(self):
        transport = example.PolicyTransport.__new__(example.PolicyTransport)
        transport.condition = __import__("threading").Condition()
        transport.frame, transport.decode_error = object(), None
        transport.observed_status = SimpleNamespace(current_action="EXTERNAL")
        with self.assertRaisesRegex(RuntimeError, "already active"):
            transport.wait_ready(1.)


class CycleTest(unittest.TestCase):
    def run_loop(self, *, live=False, fail=False, timing=False):
        clock = SimpleNamespace(now=1_700_000_000_000_000_000)
        policy = policy_fixture()

        def infer(_obs):
            clock.now += 4_000_000
            return np.zeros(2, dtype=np.float32)

        policy.infer = infer
        frame = example.StateFrame(frame_state(123), (0.,) * 31, (0.,) * 31,
                                   (0.,) * 3, (1., 0., 0., 0.), 100, 101, 102)
        transport = SimpleNamespace(latest=lambda: frame, receipt=None)
        directions = SimpleNamespace(snapshot=lambda: (), statistics=lambda: {})
        records, times = [], []

        def publish(command):
            times.append(clock.now)
            if fail:
                raise RuntimeError("publish failed")
            from dataclasses import replace
            return replace(command, sequence=1000+len(times), session_id=7)

        def sleep(delay):
            clock.now += round(delay * 1_000_000_000)

        controller = SimpleNamespace(publish=publish) if live else None
        if controller is not None and timing:
            controller._last_publish_timing = lambda: dict(publish_total_ns=123,
                outcome='raised' if fail else 'returned')
        with patch.object(example.time, "time_ns", side_effect=lambda: clock.now), \
             patch.object(example.time, "monotonic_ns", side_effect=lambda: clock.now), \
             patch.object(example.time, "sleep", side_effect=sleep), \
             patch.object(example.time, "monotonic", side_effect=AssertionError("monotonic clock used")), \
             patch.object(example.time, "perf_counter_ns", side_effect=AssertionError("perf clock used")):
            try:
                example.run_cycles(policy, transport, directions, SimpleNamespace(cycle=records.append),
                                   controller=controller, duration=.05, transition=([], 0.))
            except RuntimeError:
                if not fail:
                    raise
        return records, times

    def test_shadow_has_s0_to_s5_but_no_fabricated_publish_or_trace_keys(self):
        records, times = self.run_loop()
        self.assertFalse(times)
        for row in records:
            self.assertTrue(all(isinstance(row[f"S{i}"], int) for i in range(6)))
            self.assertEqual(row["S6"], NOT_MEASURED)
            self.assertEqual(row["sequence"], NOT_MEASURED)
            self.assertNotIn("cmd_id", row)
            self.assertEqual(len(row["target_q"]), 31)

    def test_publish_timing_is_logged_on_success_and_failure(self):
        for fail in [False, True]:
            records, _ = self.run_loop(live=True, fail=fail, timing=True)
            for row in records:
                self.assertEqual(row['publish_timing'], dict(publish_total_ns=123,
                    outcome='raised' if fail else 'returned'))

    def test_live_uses_actual_sequence_and_50_hz_schedule(self):
        records, times = self.run_loop(live=True)
        np.testing.assert_allclose(np.diff(times), [20_000_000] * (len(times) - 1))
        for i, row in enumerate(records):
            self.assertEqual(row["sequence"], 1001 + i)
            self.assertEqual(row["session_id"], 7)
            self.assertEqual(row["publish_result"], "returned")
            self.assertGreaterEqual(row["S6"], row["S5"])
            self.assertEqual(row['applied_sequence_lag'], NOT_MEASURED)
            self.assertEqual(row['control_interval_ns'], NOT_MEASURED if i == 0 else 20_000_000)

    def test_transition_drains_direction_input_on_every_tick(self):
        from dataclasses import replace
        policy = policy_fixture()
        target = policy.decode(np.zeros(2, dtype=np.float32))
        policy.infer = Mock(side_effect=AssertionError("transition inferred"))
        base = 1_700_000_000_000_000_000
        clock = SimpleNamespace(now=base)
        frame = example.StateFrame(frame_state(1), (0.,)*31, (0.,)*31,
                                   (0.,)*3, (1.,0.,0.,0.), 1, 2, 3)
        directions = SimpleNamespace(snapshot=Mock(return_value=()),
                                     statistics=Mock(return_value={"joy_pressed_counts": {0: 1}}))
        records = []
        def sleep(delay):
            clock.now += round(delay*1e9)
        with patch.object(example.time, "time_ns", side_effect=lambda: clock.now), \
             patch.object(example.time, "sleep", side_effect=sleep):
            example.run_cycles(policy, SimpleNamespace(latest=lambda: frame), directions,
                               SimpleNamespace(cycle=records.append), controller=None,
                               duration=.04, transition=(target, 1.))
        self.assertGreaterEqual(len(records), 2)
        self.assertEqual(directions.snapshot.call_count, len(records))
        self.assertEqual(directions.statistics.call_count, len(records))
        self.assertTrue(all(r["phase"] == "transition" for r in records))
        self.assertTrue(all(r["joy_pressed_counts"] == {0: 1} for r in records))
        policy.infer.assert_not_called()

    def test_policy_is_50hz_with_400hz_latest_state_source(self):
        from dataclasses import replace
        base=1_700_000_000_000_000_000
        clock=SimpleNamespace(now=base)
        source_ids=[];published=[];inferred=[];records=[]
        policy=policy_fixture()
        def latest():
            # Native keep-last ring: at each pull, return the newest of 400Hz samples.
            state_id=(clock.now-base)//2_500_000
            source_ids.append(state_id)
            state=replace(frame_state(state_id),ts_state_real_us=clock.now//1000)
            return example.StateFrame(state,(0.,)*31,(0.,)*31,(0.,)*3,(1.,0.,0.,0.),clock.now,clock.now,clock.now)
        def infer(_):
            inferred.append(clock.now)
            return np.zeros(2,dtype=np.float32)
        policy.infer=infer
        def publish(command):
            published.append(clock.now)
            return replace(command,session_id=7,sequence=len(published))
        def sleep(delay):clock.now+=round(delay*1e9)
        with patch.object(example.time,'time_ns',side_effect=lambda:clock.now),patch.object(example.time,'sleep',side_effect=sleep):
            example.run_cycles(policy,SimpleNamespace(latest=latest),SimpleNamespace(snapshot=lambda:(),statistics=lambda:{}),SimpleNamespace(cycle=records.append),controller=SimpleNamespace(publish=publish),duration=.19,transition=([],0.))
        self.assertEqual(len(inferred),11)  # t=0..200ms: original grid/bounded-loop semantics.
        self.assertEqual(len(published),len(inferred))
        self.assertEqual(len(records),len(inferred))
        np.testing.assert_array_equal(np.diff(published),[20_000_000]*(len(published)-1))
        np.testing.assert_array_equal(np.diff(source_ids[1:]),[8]*(len(published)-1))

    def test_failed_publish_records_cycle_without_fabricated_identity(self):
        records, _times = self.run_loop(live=True, fail=True)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["publish_result"], "raised")
        self.assertEqual(records[0]["sequence"], NOT_MEASURED)
        self.assertIn("publish failed", records[0]["error"])


class DurationTest(unittest.TestCase):
    def arguments(self, *extra):
        return ["--live", "--locomotion-dir", "/unused", "--log", "/unused.jsonl", *extra]

    def test_default_and_explicit_zero_are_unlimited(self):
        self.assertEqual(example.parse_args(self.arguments()).duration, 0)
        self.assertEqual(example.parse_args(self.arguments("--duration", "0")).duration, 0)
        self.assertEqual(example.parse_args(self.arguments("--duration", "60")).duration, 60)

    def test_invalid_duration_does_not_become_an_unlimited_live(self):
        for value in ("-1", "nan", "inf"):
            with self.subTest(value=value), patch("sys.stderr"), self.assertRaises(SystemExit) as error:
                example.parse_args(self.arguments("--duration", value))
            self.assertEqual(error.exception.code, 2)

    def test_unlimited_loop_exceeds_sixty_seconds_until_keyboard_interrupt(self):
        clock = SimpleNamespace(now=1_700_000_000_000_000_000)
        policy = policy_fixture()
        def infer(_observation):
            clock.now += 31_000_000_000
            return np.zeros(2, dtype=np.float32)
        policy.infer = infer
        frame = example.StateFrame(frame_state(1), (0.,)*31, (0.,)*31,
                                   (0.,)*3, (1.,0.,0.,0.), 1, 2, 3)
        records = []
        def record(row):
            records.append(row)
            if len(records) == 3:
                raise KeyboardInterrupt
        with patch.object(example.time, "time_ns", side_effect=lambda: clock.now), \
             self.assertRaises(KeyboardInterrupt):
            example.run_cycles(policy, SimpleNamespace(latest=lambda: frame),
                               SimpleNamespace(snapshot=lambda: (), statistics=lambda: {}),
                               SimpleNamespace(cycle=record), controller=None, duration=0,
                               transition=([],0.))
        self.assertEqual(len(records), 3)
        self.assertGreater(records[-1]["S5"] - records[0]["S5"], 60_000_000_000)

    def test_keyboard_interrupt_after_live_entry_runs_cancel_and_close(self):
        args = example.parse_args(self.arguments())
        controller, transport, directions, log = Mock(), Mock(), Mock(), Mock()
        transport.observed_status = SimpleNamespace(current_action="PASSIVE",
            phase=SimpleNamespace(value="IDLE"), generation=0)
        transport.publishers = {}
        transport.get_robot_info.return_value = SimpleNamespace(info=SimpleNamespace(body=SimpleNamespace(roster_version=1, joints=[])))
        controller.get_robot_info.return_value = transport.get_robot_info.return_value
        directions.node.publishers = []
        directions.statistics.return_value = {}
        controller.exit_external.return_value = SimpleNamespace(current_action="PASSIVE",
            phase=SimpleNamespace(value="IDLE"))
        with patch.object(example, "version", return_value="0.2.1"), \
             patch.object(example, "parse_args", return_value=args), \
             patch.object(example, "VelocityModePolicy", return_value=Mock(inventory={}, order=[])), \
             patch.object(example, "PolicyTransport", return_value=transport), \
             patch.object(example, "ExternalController", return_value=controller), \
             patch.object(example, "DirectionInput", return_value=directions), \
             patch.object(example, "CycleLog", return_value=log), \
             patch.object(example, "transition_duration", return_value=([],0.)), \
             patch.object(example, "run_cycles", side_effect=KeyboardInterrupt), \
             self.assertRaises(KeyboardInterrupt):
            example.main()
        controller.exit_external.assert_called_once_with(args.timeout)
        controller.close.assert_called_once_with()
        directions.close.assert_called_once_with()
        log.close.assert_called_once_with()


class TransitionTest(unittest.TestCase):
    def test_transition_uses_driven_slots_and_the_existing_profile_formula(self):
        import yaml

        policy = policy_fixture()
        policy.default_q[:] = 0.
        config = {"transition": {"min_duration_s": .2, "max_duration_s": 1.5},
                  "actions": {"HUMANOID": {"params": {
                      "transition_small_delta_skip_threshold_q": .1,
                      "transition_speed_q_per_s": 1.}}}}
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/"config").mkdir()
            (root/"config/locomotion_foot_humanoid_asample.yaml").write_text(yaml.safe_dump(config))
            for slot, delta, expected in ((0, .1, 0.), (6, .11, .2),
                                           (0, .6, .6), (6, 5., 1.5), (29, .8, .8)):
                with self.subTest(slot=slot, delta=delta):
                    q = [0.]*31
                    q[14], q[slot] = 10000., delta
                    target, duration = example.transition_duration(policy, SimpleNamespace(q=q), root)
                    self.assertEqual(len(target), 31)
                    self.assertAlmostEqual(duration, expected)
        self.assertNotIn(14, policy.driven_slots)


if __name__ == "__main__":
    unittest.main()
