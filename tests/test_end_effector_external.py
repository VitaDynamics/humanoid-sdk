"""Single CLI entry with synthetic endpoints and the original Body lifecycle.

Uses real new DTO/codec fields and the shared controller allocator. This is
regression source, not measured robot output or proof of transport delivery.
"""
from dataclasses import replace
from contextlib import contextmanager
import io
import math
import signal
import unittest
from unittest.mock import Mock, patch

from examples.end_effector import end_effector_external as demo
from locomotion_aorta import BODY_CMD_TOPIC
from locomotion_aorta import _end_effector_codec as codec
from locomotion_aorta.end_effector_types import (
    CommandKey, ControlMode, DeviceModel, EndEffectorSubmission,
    Field, InhibitReason, PositionDomain, SampleStatus,
    SampleStream,
)
from test_end_effector_codec import decode, info_object, state_object, touch_object
from test_examples import FakeClock, RecordingController
from body_fixtures import robot_info_reply


class Endpoint:
    """Deterministic sensor seam, not another permission authority."""
    def __init__(self, connection, side='right', *, hand=False,
                 stale=False, inhibited=False, touch=False, feedback_moves=True):
        self.connection, self.side = connection, side
        wire_side = 1 if side == 'left' else 2
        n = 6 if hand else 1
        info = info_object(side=wire_side, n=n, touch=touch)
        info.deviceModel = DeviceModel.BRAINCO_REVO2 if hand else DeviceModel.AGX
        info.posMax = [1.] * n if hand else [.125]
        info.velLimitMax = [1.] * n
        info.positionDomain = [PositionDomain.JOINT_ANGLE if hand else PositionDomain.TOTAL_OPENING] * n
        self.info = decode('info', info)
        self._state = decode('state', state_object(side=wire_side, n=n))
        self._touch = decode('touch', touch_object(side=wire_side)) if touch else None
        self.observation_sources = ((SampleStream.EE_STATE, wire_side),)
        self.commands, self.times, self.observations = [], [], []
        self.stale, self.inhibited = stale, inhibited
        self.feedback_moves = feedback_moves
        self.after_publish = None
        self._position_received_at = connection.clock.monotonic()

    @property
    def state(self):
        if not self.stale:
            self._state = replace(self._state, state_id=self._state.state_id+1)
        if self.commands and self.inhibited:
            self._state = replace(self._state, output_allowed=False, inhibit_reason=InhibitReason.DOMAIN_FAULT)
        return self._state

    @property
    def touch(self):
        return self._touch

    def motion_feedback(self):
        previous = self._state.state_id
        state = self.state
        if state.state_id != previous:
            self._position_received_at = self.connection.clock.monotonic()
        return state, state, state, self._position_received_at

    def wait_info(self, timeout):
        return self.info

    def wait_state(self, timeout, **kwargs):
        return self.state

    def publish(self, target, *, input_observations):
        codec.validate_observations(self.info.runtime_id, input_observations, sources=self.observation_sources)
        prepared = codec.prepare_target(self.info, target)
        context = self.connection.controller._reserve_command()
        submission = EndEffectorSubmission(self.info.runtime_id, self.info.layout_id, self.info.side,
            context.control_id, CommandKey(context.session_id, context.sequence),
            prepared, input_observations=input_observations)
        self.commands.append(submission)
        self.times.append(self.connection.clock.monotonic())
        self.observations.append(input_observations)
        if self.feedback_moves:
            # Synthetic measurement, not a claim about admission or device ACK.
            self._state = replace(self._state, pos=prepared.pos)
        if self.after_publish:
            self.after_publish(submission)
        return submission


class EndEffectorExampleTest(unittest.TestCase):
    def setUp(self):
        self.clock, self.events = FakeClock(), []

    def controller(self, **options):
        controller = RecordingController(self.clock, **options)
        self.addCleanup(controller.close)
        return controller

    def target(self, endpoint, **kwargs):
        return demo.position_target(endpoint.info,
            kwargs.pop('positions', (.0625,)*len(endpoint.info.axis_names)),
            unit='rad' if len(endpoint.info.axis_names) == 6 else 'm', **kwargs)

    def run_case(self, controller, clients, *, targets=None, **options):
        options.setdefault('native_hz', {c.side: 250 for c in clients
                                         if c.info.device_model == DeviceModel.AGX})
        options.setdefault('native_config', 'synthetic-test-effective-config')
        return demo.run(controller, clients,
            targets=targets or {c.side: self.target(c) for c in clients},
            timeout=options.pop('timeout', .3), duration=options.pop('duration', .08),
            sleep=self.clock.sleep, monotonic=self.clock.monotonic,
            emit=self.events.append, **options)

    def test_actual_entry_shares_one_owner_allocator_and_original_body_hold(self):
        c = self.controller()
        clients = [Endpoint(c, side) for side in ('left', 'right')]
        self.run_case(c, clients, body_gains='zero')
        body_commands = [v for topic,v in c.transport.published if topic == BODY_CMD_TOPIC]
        self.assertTrue(body_commands)
        for command in body_commands:
            self.assertEqual(len(command.joints), 31)
            self.assertTrue(all(j.q == (0. if i == 14 else .1) and
                (j.kp,j.kd,j.dq,j.tau) == (0.,)*4 for i,j in enumerate(command.joints)))
        keys = [CommandKey(cmd.session_id,cmd.sequence) for cmd in body_commands]
        for client in clients:
            self.assertTrue(client.commands)
            self.assertEqual(client.commands[-1].target.pos, (.0625,))
            keys.extend(cmd.key for cmd in client.commands)
            for cmd in client.commands:
                self.assertEqual(cmd.control_id, 13)
                self.assertGreaterEqual(cmd.target.pos[0], .03125)
                self.assertLessEqual(cmd.target.pos[0], .0625)
                self.assertEqual(cmd.target.vel_limit, (0.,))
                self.assertFalse(cmd.target.command_fields[0] & Field.TAU)
                self.assertEqual(cmd.input_observations[0].runtime_id, client.info.runtime_id)
                self.assertEqual((cmd.input_observations[0].stream,cmd.input_observations[0].side),
                                 (SampleStream.EE_STATE,client.info.side))
        self.assertEqual(len(keys),len(set(keys)))
        self.assertEqual({key.session_id for key in keys},{17})
        self.assertEqual(sorted(k.sequence for k in keys),list(range(1,len(keys)+1)))
        self.assertEqual(c.calls.count('enter_external'),1)
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_none_keeps_body_without_waiting_for_an_end_effector(self):
        c = self.controller()
        self.run_case(c, [])
        self.assertGreater(c.command_count,0)
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_ee_only_does_not_read_or_publish_body(self):
        c = self.controller()
        c.wait_state = Mock(side_effect=AssertionError('Body read'))
        original_info = c.get_robot_info
        c.get_robot_info = Mock(side_effect=original_info)
        e = Endpoint(c)
        self.run_case(c,[e],body_enabled=False)
        self.assertTrue(e.commands)
        self.assertEqual(c.command_count,0)
        c.wait_state.assert_not_called(); c.get_robot_info.assert_called_once()
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_mixed_rates_do_not_couple_endpoints_or_speed_up_body(self):
        c = self.controller()
        clients = [Endpoint(c,side) for side in ('left','right')]
        self.run_case(c,clients,duration=.4,rates={'left':20.,'right':100.},
                      native_hz={'left':200,'right':125})
        self.assertGreater(len(clients[1].commands),len(clients[0].commands)*3)
        for times,period in ((c.command_times,.02),(clients[0].times,.05),(clients[1].times,.01)):
            self.assertTrue(all(b-a >= period-1e-10 for a,b in zip(times,times[1:])))
        for endpoint, frequency in zip(clients, (200, 125)):
            for first, second, a, b in zip(endpoint.commands, endpoint.commands[1:], endpoint.times, endpoint.times[1:]):
                self.assertLessEqual(abs(second.target.pos[0]-first.target.pos[0]), min(b-a, 1/frequency))
            self.assertEqual(endpoint.commands[-1].target.pos, (.0625,))
        self.assertEqual(c.calls.count('enter_external'),1)

    def test_slow_host_never_replays_missed_deadlines(self):
        original_sleep = self.clock.sleep
        self.clock.sleep = lambda seconds: original_sleep(seconds+.031)
        c = self.controller(); e=Endpoint(c)
        self.run_case(c,[e],duration=.2,rates={'right':100.})
        self.assertTrue(all(b-a >= .031 for a,b in zip(e.times,e.times[1:])))
        for first, second, a, b in zip(e.commands,e.commands[1:],e.times,e.times[1:]):
            self.assertLessEqual(abs(second.target.pos[0]-first.target.pos[0]), min(b-a, 1/250))
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_agx_missing_native_config_fails_before_owner_but_other_vendor_is_not_gated(self):
        c = self.controller(); e = Endpoint(c)
        for frequencies, config in (({}, 'deployment'), ({'right': 250}, None),
                                    ({'right': 0}, 'deployment'), ({'right': True}, 'deployment')):
            with self.subTest(frequencies=frequencies, config=config), self.assertRaisesRegex(ValueError, 'native frequency'):
                self.run_case(c, [e], native_hz=frequencies, native_config=config)
            self.assertNotIn('enter_external', c.calls)
        c2 = self.controller(); hand = Endpoint(c2, hand=True)
        self.run_case(c2, [hand], native_hz={}, native_config=None)
        self.assertEqual(c2.calls.count('exit_external'), 1)

    def test_agx_unproven_velocity_limit_fails_before_owner(self):
        c = self.controller(); e = Endpoint(c)
        target = self.target(e)
        e.info = replace(e.info, valid_limits=(Field.POS | Field.TAU,))
        with self.assertRaisesRegex(ValueError, 'proven velocity limit'):
            self.run_case(c, [e], targets={'right': target})
        self.assertNotIn('enter_external', c.calls)

    def test_agx_local_submission_without_observed_motion_fails_and_cancels(self):
        for scenario in ('loss', 'busy'):
            with self.subTest(scenario=scenario):
                c = self.controller(); e = Endpoint(c, feedback_moves=False)
                with self.assertRaisesRegex(TimeoutError, 'movement not observed'):
                    self.run_case(c, [e], body_enabled=False)
                self.assertGreater(len(e.commands), 1)
                self.assertEqual(e.commands[-1].target.pos, (.0625,))
                self.assertEqual(c.calls.count('exit_external'), 1)
                self.assertFalse(any('fresh feedback observed movement' in line for line in self.events))

    def test_agx_feedback_only_in_opposite_direction_is_not_requested_motion(self):
        c = self.controller(); e = Endpoint(c, feedback_moves=False)
        e.after_publish = lambda _: setattr(e, '_state', replace(e._state, pos=(.02,)))
        with self.assertRaisesRegex(TimeoutError, 'movement not observed'):
            self.run_case(c, [e], body_enabled=False)
        self.assertEqual(c.calls.count('exit_external'), 1)
        self.assertFalse(any('fresh feedback observed movement' in line for line in self.events))

    def test_agx_short_source_interval_is_not_expanded_to_native_period(self):
        c = self.controller(); e = Endpoint(c)
        self.run_case(c, [e], body_enabled=False, duration=.08,
                      rates={'right':500.}, native_hz={'right':100})
        for first, second, a, b in zip(e.commands,e.commands[1:],e.times,e.times[1:]):
            self.assertLessEqual(abs(second.target.pos[0]-first.target.pos[0]), min(b-a, 1/100))
        self.assertEqual(e.commands[-1].target.pos, (.0625,))
        self.assertTrue(any('fresh feedback observed movement' in line and 'unconfirmed' in line
                            for line in self.events))

    def test_agx_entry_budget_uses_encoded_nonbinary_rate_and_native_float_difference(self):
        import flatbuffers
        import struct

        def f32(value):
            return struct.unpack('<f', struct.pack('<f', value))[0]

        c = self.controller(); e = Endpoint(c)
        e._state = replace(e._state, pos=(0.0,))
        e.info = replace(e.info, vel_limit_max=(f32(.03),))
        rate = .005000001008738764
        target = self.target(e, positions=(.0003,), vel_limit=(rate,))
        bindings = codec.load_bindings()
        encoded = []

        def capture(submission):
            builder = flatbuffers.Builder(128)
            builder.Finish(codec.fill_command(bindings, submission, builder, 0))
            wire = bindings.EndEffectorCmd.EndEffectorCmd.GetRootAs(builder.Output())
            encoded.append((e.times[-1], wire.Axes(0).Pos(), wire.Axes(0).VelLimit()))

        e.after_publish = capture
        self.run_case(c, [e], targets={'right': target}, body_enabled=False,
                      duration=.02, timeout=1., rates={'right':50.}, native_hz={'right':50})
        self.assertGreater(len(encoded), 2)
        self.assertEqual(encoded[1][2], f32(rate))
        self.assertGreater(encoded[1][1], encoded[0][1])
        for (t0, previous, _), (t1, position, wire_rate) in zip(encoded, encoded[1:]):
            native_difference = abs(f32(position - previous))
            self.assertLessEqual(native_difference, wire_rate * min(t1-t0, 1/50))

    def test_agx_first_nonzero_submission_pairs_feedback_baseline_and_sample(self):
        import struct

        def f32(value):
            return struct.unpack('<f', struct.pack('<f', value))[0]

        for post_submit_move in (False, True):
            with self.subTest(post_submit_move=post_submit_move):
                event_start = len(self.events)
                c = self.controller(); e = Endpoint(c, feedback_moves=False)
                drift = f32(e._state.pos[0] + .00001)
                first_used = []

                def feedback(submission):
                    if len(e.commands) == 1:
                        # Real pre-submission drift; later samples retain this value.
                        e._state = replace(e._state, pos=(drift,))
                    elif len(e.commands) == 2:
                        first_used.append((e._state.pos[0], submission.input_observations[0].sample_id))
                        if post_submit_move:
                            e._state = replace(e._state, pos=(f32(drift + .00002),))

                e.after_publish = feedback
                options = dict(body_enabled=False, timeout=.3)
                if post_submit_move:
                    self.run_case(c, [e], **options)
                    self.assertTrue(any('fresh feedback observed movement' in line and 'unconfirmed' in line
                                        for line in self.events[event_start:]))
                else:
                    with self.assertRaisesRegex(TimeoutError, 'movement not observed'):
                        self.run_case(c, [e], **options)
                    self.assertFalse(any('fresh feedback observed movement' in line for line in self.events[event_start:]))
                self.assertEqual(first_used[0][0], drift)
                self.assertGreater(e._state.state_id, first_used[0][1])
                self.assertEqual(c.calls.count('exit_external'), 1)

    def test_agx_source_clock_rollback_stops_before_next_target(self):
        c = self.controller(); e = Endpoint(c)
        e.after_publish = lambda _: setattr(self.clock, 'now', self.clock.now - .1)
        with self.assertRaisesRegex(RuntimeError, 'clock moved backwards'):
            self.run_case(c, [e], body_enabled=False)
        self.assertEqual(len(e.commands), 1)
        self.assertEqual(c.calls.count('exit_external'), 1)

    def test_agx_unfinished_final_point_is_failure_not_success(self):
        c = self.controller(); e = Endpoint(c)
        with self.assertRaisesRegex(TimeoutError, 'final source point'):
            self.run_case(c, [e], body_enabled=False, duration=.01, timeout=.03,
                          rates={'right':50.}, native_hz={'right':1000})
        self.assertTrue(e.commands)
        self.assertNotEqual(e.commands[-1].target.pos, (.0625,))
        self.assertEqual(c.calls.count('exit_external'), 1)

    def test_trajectory_entry_uses_post_prepare_state_and_keeps_original_return_target(self):
        for during in ('entry', 'touch'):
            with self.subTest(during=during):
                c = self.controller()
                clients = [Endpoint(c, side, touch=True) for side in ('left', 'right')]
                saved = {e.side: e._state.pos for e in clients}
                def changed():
                    for e in clients:
                        e._state = replace(e._state, pos=(.07,))
                original_enter = c.enter_external
                def enter(timeout):
                    status = original_enter(timeout)
                    if during == 'entry':
                        changed()
                    return status
                c.enter_external = enter
                future = Mock()
                future.done.return_value = True
                def reply():
                    changed()
                    return clients[0]._state, clients[0].touch
                future.result.side_effect = reply
                pool = Mock(); pool.submit.return_value = future
                with patch.object(demo, 'ThreadPoolExecutor', return_value=pool):
                    self.run_case(c, clients, pattern='sine', period=1., duration=.12,
                        sine_min=.01, sine_max=.08, touch='on' if during == 'touch' else 'keep')
                for e in clients:
                    self.assertEqual(e.commands[0].target.pos, (.07,))
                    self.assertNotEqual(e.commands[0].target.pos, saved[e.side])
                    self.assertEqual(e.commands[-1].target.pos, saved[e.side])
                    for a, b, t0, t1 in zip(e.commands, e.commands[1:], e.times, e.times[1:]):
                        self.assertLessEqual(abs(b.target.pos[0]-a.target.pos[0]), min(t1-t0, 1/250)+1e-9)
                self.assertEqual(c.calls.count('exit_external'), 1)

    def test_nondividing_body_and_endpoint_deadlines_keep_each_cadence(self):
        c = self.controller()
        clients = [Endpoint(c, side) for side in ('left', 'right')]
        self.run_case(c, clients, duration=.12, rates={'left': 1/.015, 'right': 1/.025})
        for times, period, minimum in ((c.command_times, .02, 7),
                                       (clients[0].times, .015, 9),
                                       (clients[1].times, .025, 6)):
            self.assertGreaterEqual(len(times), minimum)
            for index, actual in enumerate(times):
                self.assertAlmostEqual(actual-times[0], index*period, places=9)
        self.assertEqual(c.calls.count('exit_external'), 1)

    def test_slow_endpoint_deadline_survives_target_to_return_phase(self):
        c = self.controller(); e = Endpoint(c)
        saved = e._state.pos
        self.run_case(c, [e], duration=1., rates={'right': 1.}, native_hz={'right': 2}, return_to_start=True)
        returning = [i for i, command in enumerate(e.commands) if command.target.pos == saved]
        self.assertTrue(returning)
        self.assertGreater(returning[0], 0)
        self.assertTrue(all(b-a >= 1.-1e-9 for a, b in zip(e.times, e.times[1:])))
        self.assertGreaterEqual(e.times[returning[0]]-e.times[returning[0]-1], 1.-1e-9)
        self.assertEqual(e.commands[-1].target.pos, saved)
        self.assertEqual(c.calls.count('exit_external'), 1)

    def test_explicit_units_and_none_zero_signed_torque_remain_distinct(self):
        c = self.controller(); e=Endpoint(c)
        for torque in (None,(0.,),(-.25,)):
            with self.subTest(torque=torque):
                target=self.target(e,torque=torque)
                self.assertEqual(bool(target.command_fields[0]&Field.TAU),torque is not None)
                self.assertEqual(target.tau,(0.,) if torque is None else torque)
        with self.assertRaisesRegex(ValueError,'unit'):
            demo.position_target(e.info,(.05,),unit='rad')
        with self.assertRaises(ValueError): self.target(e,vel_limit=(-.1,))
        with self.assertRaises(ValueError): self.target(e,torque=(2.,))

    def test_unknown_faulted_nonfinite_wrong_ref_and_bad_target_fail_before_prepare(self):
        for changes,target_options in (({'sample_status':SampleStatus.NO_SNAPSHOT},{}),
                ({'valid_fields':(0,)},{}),({'pos':(math.nan,)},{}),
                ({'runtime_id':101},{}),
                ({'error_code':(1,)},{}),({}, {'positions':(.2,)})):
            with self.subTest(changes=changes):
                c=self.controller(); e=Endpoint(c); e._state=replace(e._state,**changes)
                with self.assertRaises((ValueError,RuntimeError)):
                    self.run_case(c,[e],targets={e.side:self.target(e,**target_options)})
                self.assertNotIn('enter_external',c.calls); self.assertFalse(e.commands)

    def test_mismatched_connection_or_K_never_takes_ownership(self):
        for wrong in ('connection','K'):
            c=self.controller(); e=Endpoint(c)
            if wrong=='connection': e.connection=self.controller()
            else: e.observation_sources=()
            with self.assertRaises(ValueError): self.run_case(c,[e])
            self.assertNotIn('enter_external',c.calls)

    def test_inhibit_and_stale_abort_once(self):
        for options in ({'inhibited':True},{'stale':True}):
            with self.subTest(options=options):
                c=self.controller(); e=Endpoint(c,**options)
                with self.assertRaises((RuntimeError,TimeoutError)):
                    self.run_case(c,[e],duration=.4)
                self.assertEqual(c.calls.count('enter_external'),1)
                self.assertEqual(c.calls.count('exit_external'),1)

    def test_descriptor_change_after_first_send_stops_and_cancels(self):
        c=self.controller(); e=Endpoint(c)
        e.after_publish=lambda _: setattr(e,'_state',replace(e._state,runtime_id=999))
        with self.assertRaises(ValueError): self.run_case(c,[e])
        self.assertEqual(len(e.commands),1)
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_hand_drift_aborts_while_gripper_drift_is_observational(self):
        for hand in (False,True):
            c=self.controller(drift_command_range=(1,200)); e=Endpoint(c,hand=hand)
            if hand:
                with self.assertRaisesRegex(RuntimeError,'held pose'): self.run_case(c,[e])
            else:
                self.run_case(c,[e])
                self.assertTrue(any('BODY_HOLD summary' in line for line in self.events))
            self.assertEqual(c.calls.count('exit_external'),1)

    def test_body_ownership_and_stale_guards_are_not_bypassed(self):
        for options in ({'reject_after_commands':1},{'fail_phase_after_commands':1},
                        {'stale_after_commands':1}):
            with self.subTest(options=options):
                c=self.controller(**options); e=Endpoint(c)
                with self.assertRaises(RuntimeError): self.run_case(c,[e],duration=.5)
                self.assertEqual(c.calls.count('exit_external'),1)

    def test_interrupt_stops_targets_without_zero_return_and_cancels_once(self):
        c=self.controller(); e=Endpoint(c)
        def interrupt(_): signal.getsignal(signal.SIGINT)(signal.SIGINT,None)
        e.after_publish=interrupt
        with self.assertRaises(KeyboardInterrupt): self.run_case(c,[e],return_to_start=True)
        self.assertEqual(len(e.commands),1)
        self.assertEqual(e.commands[0].target.pos,(.03125,))
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_unconfirmed_cancel_never_reports_passive_success(self):
        c=self.controller(cancel_action='FIXED_STAND'); e=Endpoint(c)
        with self.assertRaisesRegex(RuntimeError,'expected PASSIVE'): self.run_case(c,[e])
        self.assertTrue(any('CANCEL UNCONFIRMED' in line for line in self.events))
        self.assertFalse(any('CANCEL confirmed' in line for line in self.events))

    def test_new_state_has_no_numeric_host_target_for_duration_or_arrival(self):
        c=self.controller(); e=Endpoint(c)
        self.run_case(c,[e],return_to_start=True)
        self.assertEqual(e.commands[-1].target.pos,(.03125,))
        self.assertFalse(hasattr(e._state,'host_target'))
        self.assertTrue(any('physical completion and hold remain NOT_MEASURED' in line for line in self.events))
        self.assertFalse(any('arrival confirmed' in line for line in self.events))

    def test_sine_same_phase_return_and_peak_speed_limit(self):
        c=self.controller(); clients=[Endpoint(c,side) for side in ('left','right')]
        self.run_case(c,clients,pattern='sine',period=1.,duration=1.,sine_min=.01,sine_max=.08)
        self.assertEqual(clients[0].times,clients[1].times)
        self.assertEqual([x.target.pos for x in clients[0].commands],[x.target.pos for x in clients[1].commands])
        for e in clients:
            self.assertTrue(all(.01 <= command.target.pos[0] <= .08 for command in e.commands))
            self.assertEqual(e.commands[-1].target.pos,(.03125,))
        c2=self.controller(); clients2=[Endpoint(c2,side) for side in ('left','right')]
        with self.assertRaisesRegex(ValueError,'peak command slope'):
            self.run_case(c2,clients2,pattern='sine',period=.01)
        self.assertNotIn('enter_external',c2.calls)

    def test_fraction_patterns_keep_actual_axis_ranges_and_sequential_is_single_axis(self):
        for pattern in ('simultaneous','sequential'):
            c=self.controller(); clients=[Endpoint(c,side,hand=True) for side in ('left','right')]
            # Different actual descriptor ranges per axis, no shared fake min/max.
            for e in clients:
                e.info=replace(e.info,pos_max=tuple(1.+i/10 for i in range(6)))
            self.run_case(c,clients,pattern=pattern,low_frac=.25,high_frac=.75)
            for e in clients:
                low=tuple(.25*x for x in e.info.pos_max); high=tuple(.75*x for x in e.info.pos_max)
                self.assertEqual(e.commands[-1].target.pos,low)
                self.assertTrue(all(all(0 <= q <= hi for q,hi in zip(cmd.target.pos,e.info.pos_max)) for cmd in e.commands))
                for i in range(6): self.assertTrue(any(abs(cmd.target.pos[i]-high[i]) < 1e-10 for cmd in e.commands))
            if pattern=='sequential':
                for left,right in zip(clients[0].commands,clients[1].commands):
                    above=sum(q>lo+1e-10 for e,cmd in zip(clients,(left,right))
                              for q,lo in zip(cmd.target.pos,tuple(.25*x for x in e.info.pos_max)))
                    self.assertLessEqual(above,1)
            self.assertEqual(c.calls.count('exit_external'),1)

    def test_angular_default_is_float32_twenty_degrees_and_print_rate_is_bounded(self):
        c=self.controller(); e=Endpoint(c,hand=True)
        target=self.target(e)
        self.assertAlmostEqual(target.vel_limit[0],math.radians(20),places=7)
        with self.assertRaisesRegex(ValueError,'at least 1 second'):
            self.run_case(c,[e],feedback_interval=.5)
        self.assertNotIn('enter_external',c.calls)

    def test_fraction_loop_repeats_then_signal_cancels_without_a_tail_target(self):
        for pattern in ('simultaneous', 'sequential'):
            for signum in (signal.SIGINT, signal.SIGTERM):
                with self.subTest(pattern=pattern, signum=signum):
                    c = self.controller(); e = Endpoint(c, hand=True)
                    previous_handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
                    high_visits, previous_high, stopped = 0, False, []
                    def stop_in_second_round(command):
                        nonlocal high_visits, previous_high
                        high = command.target.pos[0] == .75
                        if high and not previous_high:
                            high_visits += 1
                        previous_high = high
                        if high_visits == 2:
                            stopped.append(len(e.commands))
                            signal.getsignal(signum)(signum, None)
                    e.after_publish = stop_in_second_round
                    original_exit = c.exit_external
                    def cancel(timeout):
                        self.assertEqual(signal.getsignal(signal.SIGINT), signal.SIG_IGN)
                        self.assertEqual(signal.getsignal(signal.SIGTERM), signal.SIG_IGN)
                        return original_exit(timeout)
                    with patch.object(c, 'exit_external', side_effect=cancel):
                        with self.assertRaises(KeyboardInterrupt):
                            self.run_case(c, [e], pattern=pattern, loop=True)
                    self.assertEqual(high_visits, 2)
                    self.assertEqual(stopped, [len(e.commands)])
                    self.assertEqual(c.calls.count('enter_external'), 1)
                    self.assertEqual(c.calls.count('exit_external'), 1)
                    self.assertEqual(c.status.current_action, 'PASSIVE')
                    self.assertEqual({s: signal.getsignal(s) for s in previous_handlers}, previous_handlers)

    def test_failed_entry_attempts_cancel_once_before_any_ee_target(self):
        c = self.controller(); e = Endpoint(c, hand=True)
        with patch.object(c, 'enter_external', side_effect=RuntimeError('entry failed')), \
             patch.object(c, 'exit_external', return_value=Mock(current_action='PASSIVE')) as cancel:
            with self.assertRaisesRegex(RuntimeError, 'entry failed'):
                self.run_case(c, [e])
            cancel.assert_called_once()
        self.assertEqual(e.commands, [])

    def test_feedback_preserves_validity_without_retired_diagnostics(self):
        c=self.controller(); e=Endpoint(c)
        e._state=replace(e._state,pos=(math.nan,),valid_fields=(0,))
        demo.print_feedback(e,self.events.append)
        text='\n'.join(self.events)
        self.assertIn('pos=(nan,)',text); self.assertIn('valid=(0,)',text)
        self.assertNotIn('progress_complete=',text); self.assertNotIn('raw_pos=',text)

    def test_cli_default_and_dry_run_borrow_readonly_source_without_controller(self):
        for flags in ([],['--dry-run']):
            source,connection,client=Mock(),Mock(),Mock()
            source.return_value.__enter__=Mock(return_value=connection)
            source.return_value.__exit__=Mock(return_value=False)
            connection.get_robot_info.return_value=robot_info_reply()
            client.return_value.__enter__=Mock(return_value=Mock(side='right'))
            client.return_value.__exit__=Mock(return_value=False)
            controller=Mock()
            with patch.object(demo,'observe') as observe:
                demo.main(['--side','right']+flags,_state_source_factory=source,
                          _client_factory=client,_controller_factory=controller)
            controller.assert_not_called(); observe.assert_called_once()
            connection.get_robot_info.assert_called_once_with(5.0)
            client.assert_called_once_with('right',connection=connection,read_only=True,observation_sources=())

    def test_cli_none_does_not_construct_an_endpoint_or_request_control(self):
        source,connection,client,controller=Mock(),Mock(),Mock(),Mock()
        source.return_value.__enter__=Mock(return_value=connection)
        source.return_value.__exit__=Mock(return_value=False)
        connection.get_robot_info.return_value=robot_info_reply()
        with patch('builtins.print') as printed:
            demo.main(['--side','none'],_state_source_factory=source,
                      _client_factory=client,_controller_factory=controller)
        self.assertEqual(sum(str(call.args[0]).startswith('Body Info:') for call in printed.call_args_list), 1)
        connection.get_robot_info.assert_called_once_with(5.0)
        client.assert_not_called(); controller.assert_not_called()
        connection.wait_state.assert_called_once_with(5.0)

    def test_cli_missing_runtime_does_not_construct_endpoint_or_request_control(self):
        source, connection, client, controller = Mock(), Mock(), Mock(), Mock()
        source.return_value.__enter__ = Mock(return_value=connection)
        source.return_value.__exit__ = Mock(return_value=False)
        connection.get_robot_info.return_value = replace(robot_info_reply(), runtime_id=0)
        with self.assertRaisesRegex(RuntimeError, 'startup identity'):
            demo.main(['--side', 'right'], _state_source_factory=source,
                      _client_factory=client, _controller_factory=controller)
        client.assert_not_called()
        controller.assert_not_called()

    def test_cli_bad_options_fail_before_any_factory_or_rpc(self):
        cases=(['--ee-only','--side','none'],['--dry-run','--touch','on'],
               ['--confirm-motion','--side','right'],['--confirm-motion','--dry-run'],
               ['--right-vel-limit','-1'],['--duration','nan'],['--loop'],
               ['--side','left','--pattern','sine'],['--acceleration-limit-m-s2','1'],
               ['--period-s','0'],['--low-frac','.8','--high-frac','.2'],['--touch','invalid'])
        for flags in cases:
            with self.subTest(flags=flags),patch('sys.stderr',new_callable=io.StringIO):
                source,client,controller=Mock(),Mock(),Mock()
                with self.assertRaises(SystemExit):
                    demo.main(flags,_state_source_factory=source,_client_factory=client,
                              _controller_factory=controller)
                source.assert_not_called(); client.assert_not_called(); controller.assert_not_called()

    def test_main_and_run_share_finite_motion_parameter_validation(self):
        source,connection,client=Mock(),Mock(),Mock()
        source.return_value.__enter__=Mock(return_value=connection)
        source.return_value.__exit__=Mock(return_value=False)
        connection.get_robot_info.return_value=robot_info_reply()
        client.return_value.__enter__=Mock(return_value=Mock(side='right'))
        client.return_value.__exit__=Mock(return_value=False)
        with patch.object(demo,'_validate_motion_parameters',wraps=demo._validate_motion_parameters) as validate:
            demo.main(['--side','right'],_state_source_factory=source,
                      _client_factory=client,_controller_factory=Mock())
        self.assertEqual(1,validate.call_count)
        controller=self.controller(); endpoint=Endpoint(controller)
        target=demo.position_target(endpoint.info,(.0625,),unit='m')
        with patch.object(demo,'_validate_motion_parameters',side_effect=ValueError('shared validation')) as validate:
            with self.assertRaisesRegex(ValueError,'shared validation'):
                demo.run(controller,[endpoint],{endpoint.side:target},native_hz={'right':250},
                         native_config='synthetic-test-effective-config')
        self.assertEqual(1,validate.call_count)
        self.assertEqual([],endpoint.commands)
        self.assertEqual(0,controller.calls.count('enter_external'))


    def test_actual_client_publish_writes_new_root_K_and_same_body_allocator(self):
        from locomotion_aorta.end_effector import EndEffectorClient
        from test_end_effector_client import Node, confirm_info
        clock_patch=patch('locomotion_aorta.end_effector.time.monotonic',self.clock.monotonic)
        clock_patch.start(); self.addCleanup(clock_patch.stop)
        recorded=self.controller()
        controller=recorded.controller
        controller.get_robot_info(.1)
        node=Node(); recorded.transport.node=node
        clients=[]
        for side,wire_side in (('left',1),('right',2)):
            client=EndEffectorClient(side,connection=controller,
                observation_sources=((SampleStream.EE_STATE,wire_side),))
            self.addCleanup(client.close); clients.append(client)
            info=info_object(side=wire_side); info.deviceModel=DeviceModel.AGX
            confirm_info(client,node,info); node.emit(side,'state',state_object(side=wire_side))
        original_factory=node.create_publisher_typed
        samples={'left':1,'right':1}
        def publisher(*args,**kwargs):
            result=original_factory(*args,**kwargs)
            original_publish=result.publish_typed
            def publish(fill):
                original_publish(fill)
                wire=node.commands[-1][1]
                side='left' if wire.Side()==1 else 'right'
                samples[side]+=1
                state=state_object(side=wire.Side(),sample=samples[side])
                node.emit(side,'state',state)
            result.publish_typed=publish
            return result
        node.create_publisher_typed=publisher
        enter,exit_=controller.enter_external,controller.exit_external
        with patch.object(controller,'wait_state',recorded.wait_state,create=True), \
             patch.object(controller,'enter_external',wraps=enter) as entered, \
             patch.object(controller,'exit_external',wraps=exit_) as cancelled:
            demo.run(controller,clients,{c.side:demo.position_target(c.info,(.03125,),unit='m') for c in clients},
                duration=.08,timeout=.3,native_hz={'left':250,'right':250},
                native_config='synthetic-test-effective-config',sleep=self.clock.sleep,
                monotonic=self.clock.monotonic,emit=self.events.append)
        entered.assert_called_once(); cancelled.assert_called_once()
        keys=[(value.session_id,value.sequence) for topic,value in recorded.transport.published if topic==BODY_CMD_TOPIC]
        for topic,wire in node.commands:
            self.assertIn('/end_effector/',topic)
            keys.append((wire.Key().SessionId(),wire.Key().Sequence()))
            self.assertEqual(wire.InputObservationsLength(),1)
            sample=wire.InputObservations(0)
            self.assertEqual((sample.Stream(),sample.Side()),(SampleStream.EE_STATE,wire.Side()))
            self.assertEqual(wire.RuntimeId(),100)
            self.assertFalse(hasattr(sample, 'DescriptorRef'))
            self.assertEqual(wire.ControlId(),13)
            self.assertEqual(wire.ProtocolVersion(),2)
        self.assertEqual(len(keys),len(set(keys)))
        self.assertEqual(sorted(sequence for _,sequence in keys),list(range(1,len(keys)+1)))

    def test_service_wait_keeps_body_pacing_and_cancel_precedes_worker_join(self):
        # The RPC/confirmation function has separate real-interface tests below.
        # A pending service future makes the actual CLI's lifecycle observable
        # without a timing race or a second fake permission implementation.
        for failure in (False,True):
            with self.subTest(failure=failure):
                c=self.controller(); e=Endpoint(c,touch=True)
                order=[]
                future=Mock(); future.done.side_effect=[False,False,False,True]
                if failure: future.result.side_effect=OSError('service failed')
                else: future.result.return_value=(replace(e._state,state_id=2,touch_enabled=True),e.touch)
                pool=Mock(); pool.submit.return_value=future
                pool.shutdown.side_effect=lambda **kwargs: order.append(('join',c.calls.count('exit_external')))
                with patch.object(demo,'ThreadPoolExecutor',return_value=pool) as factory:
                    if failure:
                        with self.assertRaisesRegex(OSError,'service failed'): self.run_case(c,[e],touch='on')
                    else: self.run_case(c,[e],touch='on')
                factory.assert_called_once_with(max_workers=1)
                pool.submit.assert_called_once()
                self.assertIs(pool.submit.call_args.args[0],demo.set_touch_and_observe)
                self.assertGreaterEqual(c.command_count,3)
                self.assertEqual(order,[('join',1)])
                pool.shutdown.assert_called_once_with(wait=True,cancel_futures=True)
                if failure: self.assertFalse(e.commands)

    def test_pending_service_faults_cancel_then_join_then_close_without_motion_tail(self):
        for fault in ('revoked', 'stale', 'signal'):
            with self.subTest(fault=fault):
                clock = FakeClock()
                c = RecordingController(clock)
                e = Endpoint(c, touch=True)
                order = []
                pending = False
                interrupted = False
                future = Mock(); future.done.return_value = False
                pool = Mock()
                def submit(*_args, **_kwargs):
                    nonlocal pending
                    pending = True
                    if fault == 'revoked':
                        c.fail_phase_after_commands = 1
                    elif fault == 'stale':
                        c.advance_lowstate = False
                    return future
                pool.submit.side_effect = submit
                pool.shutdown.side_effect = lambda **_kw: order.append(('join', c.command_count))
                original_cancel = c.exit_external
                def cancel(timeout):
                    result = original_cancel(timeout)
                    order.append(('cancel', c.command_count))
                    return result
                c.exit_external = cancel
                def sleep(seconds):
                    nonlocal interrupted
                    clock.sleep(seconds)
                    if pending and fault == 'signal' and not interrupted:
                        interrupted = True
                        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
                @contextmanager
                def controller_factory(**_kwargs):
                    try:
                        yield c
                    finally:
                        order.append(('controller-close', c.command_count))
                        c.close()
                @contextmanager
                def client_factory(*_args, **_kwargs):
                    try:
                        yield e
                    finally:
                        order.append(('client-close', c.command_count))
                original_run = demo.run
                def run_with_clock(*args, **kwargs):
                    return original_run(*args, **kwargs, sleep=sleep,
                                        monotonic=clock.monotonic, emit=self.events.append)
                argv = ['--confirm-motion', '--side', 'right', '--right-pos', '.0625',
                        '--right-unit', 'm', '--touch', 'on', '--timeout', '.3']
                with patch.object(demo, 'ThreadPoolExecutor', return_value=pool), \
                     patch.object(demo, 'run', side_effect=run_with_clock), patch('builtins.print'):
                    if fault == 'signal':
                        demo.main(argv, _controller_factory=controller_factory, _client_factory=client_factory)
                    else:
                        message = 'ownership/phase' if fault == 'revoked' else 'lowstate became stale'
                        with self.assertRaisesRegex(RuntimeError, message):
                            demo.main(argv, _controller_factory=controller_factory, _client_factory=client_factory)
                self.assertTrue(pending)
                self.assertEqual([name for name, _ in order],
                                 ['cancel', 'join', 'client-close', 'controller-close'])
                self.assertEqual(len({count for _, count in order}), 1)
                self.assertEqual(c.calls.count('exit_external'), 1)
                self.assertFalse(e.commands)
                future.result.assert_not_called()
                pool.submit.assert_called_once()
                pool.shutdown.assert_called_once_with(wait=True, cancel_futures=True)


class TouchObservationTest(unittest.TestCase):
    def make_client(self):
        c=Mock()
        c.state=decode('state',state_object())
        c.touch=decode('touch',touch_object())
        return c

    def test_rpc_success_waits_new_valid_state_then_real_touch_not_info(self):
        c=self.make_client(); initial=c.state; old=c.touch
        clock=FakeClock()
        c.set_touch.side_effect=lambda *_a,**_k: setattr(c,'state',replace(
            initial,ts_state_pub_us=120,status_valid_mask=7,touch_enabled=True))
        def sleep(seconds):
            clock.sleep(seconds)
            c.state=replace(initial,ts_state_pub_us=130,touch_enabled=True)
        c.wait_touch.return_value=replace(old,touch_state_id=2)
        state,touch=demo.set_touch_and_observe(c,True,1.,monotonic=clock.monotonic,sleep=sleep)
        c.set_touch.assert_called_once_with(True,timeout=1.)
        self.assertEqual(state.sample_ref,initial.sample_ref); self.assertEqual(touch.touch_state_id,2)
        self.assertEqual(state.ts_state_pub_us,130)
        c.wait_state.assert_not_called()
        self.assertEqual(c.wait_touch.call_args.kwargs['after_sample'],old.sample_ref)
        c.wait_info.assert_not_called()

    def test_off_requires_state_confirmation_but_not_an_impossible_new_touch(self):
        c=self.make_client(); initial=c.state
        c.set_touch.side_effect=lambda *_a,**_k:setattr(c,'state',replace(initial,ts_state_pub_us=120,touch_enabled=False))
        _,touch=demo.set_touch_and_observe(c,False,1.)
        self.assertIsNone(touch); c.wait_touch.assert_not_called()
        c.set_touch.assert_called_once()

    def test_wrong_touch_or_state_ref_and_rpc_failure_do_not_retry(self):
        for failure in ('state','touch','rpc'):
            c=self.make_client(); updated=replace(c.state,ts_state_pub_us=120,touch_enabled=True)
            c.wait_touch.return_value=replace(c.touch,touch_state_id=2)
            if failure=='state': updated=replace(updated,runtime_id=2)
            elif failure=='touch': c.wait_touch.return_value=replace(c.touch,runtime_id=2)
            c.set_touch.side_effect=(OSError('RPC failure') if failure=='rpc' else
                                    lambda *_a,**_k:setattr(c,'state',updated))
            with self.assertRaises((ValueError,OSError)): demo.set_touch_and_observe(c,True,1.)
            c.set_touch.assert_called_once()

    def test_rpc_consumes_the_same_budget_no_full_timeout_restart(self):
        clock=FakeClock(); c=self.make_client()
        def rpc(*_a,**_k):
            clock.sleep(.8)
            c.state=replace(c.state,ts_state_pub_us=120,touch_enabled=True)
        c.set_touch.side_effect=rpc
        c.wait_touch.side_effect=TimeoutError('no Touch')
        with self.assertRaises(TimeoutError): demo.set_touch_and_observe(c,True,1.,monotonic=clock.monotonic)
        c.wait_state.assert_not_called()
        self.assertAlmostEqual(c.wait_touch.call_args.args[0],.2)
        c.set_touch.assert_called_once()

    def test_touch_confirmation_cannot_use_old_publication_or_invalid_new_touch(self):
        clock=FakeClock(); c=self.make_client()
        initial=c.state
        c.set_touch.side_effect=lambda *_a,**_k:setattr(c,'state',replace(initial,touch_enabled=True))
        with self.assertRaises(TimeoutError):
            demo.set_touch_and_observe(c,True,.03,monotonic=clock.monotonic,sleep=clock.sleep)
        c.wait_touch.assert_not_called()
        c=self.make_client(); old=c.touch
        c.set_touch.side_effect=lambda *_a,**_k:setattr(c,'state',replace(c.state,ts_state_pub_us=120,touch_enabled=True))
        invalid=replace(old,touch_state_id=2,region_stamps=tuple(replace(r,validity=64) for r in old.region_stamps))
        c.wait_touch.side_effect=[invalid,TimeoutError('no actual new valid Touch')]
        with self.assertRaises(TimeoutError):
            demo.set_touch_and_observe(c,True,1.,monotonic=clock.monotonic)
        self.assertEqual(c.wait_touch.call_args.kwargs['after_sample'],invalid.sample_ref)


class PartialFeedbackGuardTest(unittest.TestCase):
    def setUp(self):
        from test_end_effector_client import Node, confirm_info
        from test_controller import ControllerHarness
        from locomotion_aorta.end_effector import EndEffectorClient
        self.clock=FakeClock()
        self.harness=ControllerHarness(runtime_id=100); self.addCleanup(self.harness.controller.close)
        self.node=Node(); self.harness.transport.node=self.node
        self.client=EndEffectorClient('left',connection=self.harness.controller,
            observation_sources=((SampleStream.EE_STATE,1),))
        self.addCleanup(self.client.close)
        self.clock_patch=patch('locomotion_aorta.end_effector.time.monotonic',self.clock.monotonic)
        self.clock_patch.start(); self.addCleanup(self.clock_patch.stop)
        info=info_object(); info.deviceModel=DeviceModel.LINKER_L6
        confirm_info(self.client,self.node,info)

    def emit(self,sample,fields,**changes):
        state=state_object(sample=sample); state.axes[0].validFields=fields
        for name,value in changes.items():
            if name in ('pos', 'errorCode'):
                setattr(state.axes[0], name, value[0])
            else:
                setattr(state,name,value)
        self.node.emit('left','state',state)
        return self.client.state

    def guard(self):
        return demo._EndpointGuard(self.client,self.client.info,1.,self.clock.monotonic)

    def test_partial_diagnosis_keeps_position_ref_without_inventing_admission(self):
        position=self.emit(1,Field.POS)
        guard=self.guard()
        self.clock.sleep(.01)
        self.emit(2,Field.ERROR_CODE)
        self.emit(3,Field.TEMPERATURE)
        self.assertIs(guard.feedback(True),position)
        self.harness.accept_enter(); self.harness.controller.enter_external(timeout=.1)
        command=guard.publish(demo.position_target(self.client.info,(.05,),unit='m'),position)
        self.emit(4,Field.TEMPERATURE)
        self.assertIs(guard.feedback(True),position)
        self.assertFalse(hasattr(guard, 'admitted'))
        self.assertEqual(command.input_observations,(position.sample_ref,))
        newer=self.emit(5,Field.POS,pos=[.04])
        self.assertIs(guard.feedback(True),newer)

    def test_diagnostics_and_same_sample_publications_never_extend_position_deadline(self):
        position=self.emit(1,Field.POS)
        guard=self.guard()
        self.clock.sleep(demo.body.LOWSTATE_WINDOW_S/2)
        self.emit(2,Field.TEMPERATURE)
        self.emit(2,Field.TEMPERATURE,tsStatePubUs=120)
        self.clock.sleep(demo.body.LOWSTATE_WINDOW_S/2)
        with self.assertRaisesRegex(RuntimeError,'position became stale'):
            guard.feedback(True)

    def test_fault_is_not_hidden_by_temperature_or_new_position(self):
        position=self.emit(1,Field.POS)
        guard=self.guard()
        self.emit(2,Field.ERROR_CODE,errorCode=[17])
        self.emit(3,Field.TEMPERATURE)
        self.emit(4,Field.POS)
        with self.assertRaisesRegex(RuntimeError,'nonzero'):
            guard.feedback(True)

    def test_same_id_permission_revocation_and_producer_stale_stop_immediately(self):
        position=self.emit(1,Field.POS)
        guard=self.guard()
        self.emit(1,Field.POS,tsStatePubUs=120,outputAllowed=False,inhibitReason=InhibitReason.DOMAIN_FAULT)
        with self.assertRaisesRegex(RuntimeError,'inhibited'):
            guard.feedback(True)
        self.emit(2,Field.POS,sampleStatus=SampleStatus.STALE)
        with self.assertRaisesRegex(ValueError,'measured position'):
            guard.feedback(True)

    def test_preflight_waits_only_original_deadline_for_real_position(self):
        self.emit(1,Field.TEMPERATURE)
        def sleep(seconds):
            self.clock.sleep(seconds)
            self.emit(2,Field.POS)
        position=demo.wait_position(self.client,self.client.info,.03,sleep=sleep,monotonic=self.clock.monotonic)
        self.assertEqual(position.state_id,2)
        from test_end_effector_client import confirm_info
        self.node.emit('left','info',info_object(runtime=101))
        self.assertEqual(self.client.info.runtime_id, 100)
        self.harness.transport.get_robot_info = lambda timeout: replace(
            robot_info_reply(), runtime_id=101)
        with self.assertRaises(ValueError):
            self.harness.controller.get_robot_info(.1)
        with self.assertRaises(ValueError):
            demo.wait_position(self.client,self.client.info,.03,sleep=self.clock.sleep,monotonic=self.clock.monotonic)


class VendorExamplesTest(unittest.TestCase):
    """Real shared entry/run with synthetic transport, not device evidence."""
    def setup_case(self):
        self.clock = FakeClock()
        self.control = RecordingController(self.clock)
        self.addCleanup(self.control.close)
        self.endpoints, self.factories, self.events = {}, [], []

    def endpoint(self, side, model):
        endpoint = Endpoint(self.control, side, hand=model != DeviceModel.AGX)
        n = len(endpoint.info.axis_names)
        commands = Field.POS
        states = Field.POS | Field.TEMPERATURE | Field.ERROR_CODE
        limits = Field.POS
        if model == DeviceModel.BRAINCO_REVO2:
            commands |= Field.VEL_LIMIT
            states = Field.POS | Field.VEL | Field.CURRENT
            limits |= Field.VEL_LIMIT
        elif model == DeviceModel.AGX:
            commands |= Field.VEL_LIMIT | Field.TAU
            states = Field.POS | Field.TAU | Field.ERROR_CODE
            limits |= Field.VEL_LIMIT | Field.TAU
        endpoint.info = replace(endpoint.info, device_model=model,
            supported_modes=((1 << ControlMode.POSITION) |
                ((1 << ControlMode.POSITION_TORQUE) if model == DeviceModel.AGX else 0),) * n,
            supported_command_fields=(commands,) * n, supported_state_fields=(states,) * n,
            valid_limits=(limits,) * n,
            vel_min=endpoint.info.vel_min if states & Field.VEL else (),
            vel_max=endpoint.info.vel_max if states & Field.VEL else (),
            vel_limit_max=endpoint.info.vel_limit_max if commands & Field.VEL_LIMIT else (),
            tau_min=endpoint.info.tau_min if commands & Field.TAU else (),
            tau_max=endpoint.info.tau_max if commands & Field.TAU else (),
            current_min=endpoint.info.current_min if states & Field.CURRENT else (),
            current_max=endpoint.info.current_max if states & Field.CURRENT else ())
        endpoint._state = replace(endpoint._state, valid_fields=(states,) * n)
        codec.validate_info(endpoint.info)
        codec.validate_state(endpoint.info, endpoint._state)
        endpoint.set_touch = Mock(side_effect=AssertionError('unexpected Touch RPC'))
        return endpoint

    def motion_flags(self, model, side='right'):
        sides = ('left', 'right') if side == 'both' else (side,)
        flags = ['--side', side, '--ee-only', '--confirm-motion', '--timeout', '.2', '--duration', '.04']
        for selected in sides:
            flags += ['--' + selected + '-unit', 'm' if model == DeviceModel.AGX else 'rad',
                      '--' + selected + '-pos'] + ['.0625'] * (1 if model == DeviceModel.AGX else 6)
            if model == DeviceModel.AGX:
                flags += ['--' + selected + '-native-hz', '250']
        if model == DeviceModel.AGX:
            flags += ['--native-config', 'synthetic-test-effective-config']
        return flags

    def invoke(self, expected, flags, *, models=None, mutate=None):
        models = {} if models is None else models

        @contextmanager
        def connection(kind, **options):
            self.factories.append(kind)
            yield self.control

        @contextmanager
        def client(side, *, connection, read_only, observation_sources):
            self.assertIs(connection, self.control)
            endpoint = self.endpoint(side, models.get(side, expected))
            self.endpoints[side] = endpoint
            self.assertEqual(() if read_only else endpoint.observation_sources, observation_sources)
            if mutate is not None:
                mutate(endpoint)
            yield endpoint

        actual_run = demo.run
        def run(*args, **kwargs):
            kwargs.update(sleep=self.clock.sleep, monotonic=self.clock.monotonic, emit=self.events.append)
            return actual_run(*args, **kwargs)

        with patch.object(demo, 'run', side_effect=run) as executed, patch('sys.stdout', new_callable=io.StringIO) as output:
            self.executed = executed
            try:
                demo.main(flags, _expected_device_model=expected, _client_factory=client,
                          _controller_factory=lambda **kw: connection('controller', **kw),
                          _state_source_factory=lambda **kw: connection('source', **kw))
            finally:
                self.output = output.getvalue()

    def assert_no_control(self):
        self.assertEqual([], self.control.calls)
        self.assertEqual(0, self.control.command_count)
        self.assertEqual(1, self.control.controller._next_external_sequence)
        for endpoint in self.endpoints.values():
            self.assertEqual([], endpoint.commands)
            endpoint.set_touch.assert_not_called()

    def test_vendor_models_reach_shared_main_and_original_run(self):
        for model in (DeviceModel.BRAINCO_REVO2, DeviceModel.AGX, DeviceModel.LINKER_L6):
            with self.subTest(model=model):
                self.setup_case()
                self.invoke(model, self.motion_flags(model))
                self.executed.assert_called_once()
                self.assertEqual(['controller'], self.factories)
                self.assertEqual(1, self.control.calls.count('enter_external'))
                self.assertEqual(1, self.control.calls.count('exit_external'))
                self.assertEqual(0, self.control.command_count)
                endpoint = self.endpoints['right']
                self.assertTrue(endpoint.commands)
                if model == DeviceModel.LINKER_L6:
                    self.assertEqual((Field.POS,) * 6, endpoint.info.supported_command_fields)
                    self.assertEqual((), endpoint.info.vel_limit_max)
                    self.assertEqual((), endpoint.info.tau_min)
                    self.assertTrue(all(command.target.vel_limit == () and command.target.tau == ()
                                        for command in endpoint.commands))

    def test_vendor_default_and_dry_run_keep_readonly_diagnostics(self):
        for model in (DeviceModel.BRAINCO_REVO2, DeviceModel.AGX, DeviceModel.LINKER_L6):
            for option in ([], ['--dry-run']):
                with self.subTest(model=model, option=option):
                    self.setup_case()
                    def unavailable(endpoint):
                        endpoint.info = replace(endpoint.info, valid_limits=(0,) * len(endpoint.info.axis_names))
                        endpoint._state = replace(endpoint._state, ready=False, output_allowed=False,
                            inhibit_reason=InhibitReason.NOT_READY)
                    self.invoke(model, ['--side', 'right'] + option, mutate=unavailable)
                    self.assertEqual(['source'], self.factories)
                    self.executed.assert_not_called()
                    self.assertIn('Info=', self.output)
                    self.assert_no_control()

    def test_vendor_side_model_shape_and_domain_fail_before_control(self):
        for fault in ('side', 'model', 'shape', 'domain'):
            with self.subTest(fault=fault):
                self.setup_case()
                def wrong(endpoint):
                    if fault == 'side':
                        endpoint.info = replace(endpoint.info, side=1)
                    elif fault == 'domain':
                        endpoint.info = replace(endpoint.info, position_domain=(PositionDomain.TOTAL_OPENING,) * 6)
                    elif fault == 'shape':
                        endpoint.info = replace(endpoint.info, device_model=DeviceModel.BRAINCO_REVO2)
                models = {'right': DeviceModel.AGX} if fault in ('model', 'shape') else None
                with self.assertRaises(ValueError):
                    self.invoke(DeviceModel.BRAINCO_REVO2, self.motion_flags(DeviceModel.BRAINCO_REVO2),
                                models=models, mutate=wrong)
                self.executed.assert_not_called()
                self.assert_no_control()

    def test_vendor_both_checks_second_before_first_takes_control(self):
        self.setup_case()
        with self.assertRaises(ValueError):
            self.invoke(DeviceModel.AGX, self.motion_flags(DeviceModel.AGX, 'both'),
                        models={'left': DeviceModel.AGX, 'right': DeviceModel.BRAINCO_REVO2})
        self.assertEqual(['left', 'right'], list(self.endpoints))
        self.executed.assert_not_called()
        self.assert_no_control()

    def test_vendor_motion_limits_units_and_unsupported_fields_fail_before_control(self):
        for fault in ('unproven', 'outside', 'unit', 'l6_velocity', 'l6_torque'):
            with self.subTest(fault=fault):
                self.setup_case()
                model = DeviceModel.LINKER_L6 if fault.startswith('l6') else DeviceModel.AGX
                flags = self.motion_flags(model)
                if fault == 'outside':
                    flags[flags.index('--right-pos') + 1] = '.25'
                elif fault == 'unit':
                    flags[flags.index('--right-unit') + 1] = 'rad'
                elif fault.startswith('l6'):
                    flags += ['--right-vel-limit' if fault == 'l6_velocity' else '--right-torque-nm'] + ['0'] * 6
                def unproven(endpoint):
                    if fault == 'unproven':
                        endpoint.info = replace(endpoint.info, valid_limits=(Field.VEL_LIMIT | Field.TAU,))
                with self.assertRaises(ValueError):
                    self.invoke(model, flags, mutate=unproven)
                self.executed.assert_not_called()
                self.assert_no_control()

    def test_vendor_none_and_unsupported_patterns_fail_before_factories(self):
        cases = ((DeviceModel.BRAINCO_REVO2, ['--side', 'none']),
                 (DeviceModel.BRAINCO_REVO2, ['--pattern', 'sine']),
                 (DeviceModel.AGX, ['--pattern', 'sequential']),
                 (DeviceModel.LINKER_L6, ['--pattern', 'simultaneous']),
                 (DeviceModel.LINKER_L6, ['--pattern', 'sine']))
        for model, flags in cases:
            with self.subTest(model=model, flags=flags), patch('sys.stderr', new_callable=io.StringIO):
                client, controller, source = Mock(), Mock(), Mock()
                with self.assertRaises(SystemExit):
                    demo.main(flags, _expected_device_model=model, _client_factory=client,
                              _controller_factory=controller, _state_source_factory=source)
                client.assert_not_called(); controller.assert_not_called(); source.assert_not_called()

    def test_vendor_unselected_side_is_not_created(self):
        self.setup_case()
        self.invoke(DeviceModel.AGX, ['--side', 'left', '--dry-run'])
        self.assertEqual(['left'], list(self.endpoints))
        self.assert_no_control()

    def test_vendor_same_model_both_share_original_allocator(self):
        self.setup_case()
        self.invoke(DeviceModel.AGX, self.motion_flags(DeviceModel.AGX, 'both'))
        sequences = [command.key.sequence for endpoint in self.endpoints.values() for command in endpoint.commands]
        self.assertTrue(self.endpoints['left'].commands and self.endpoints['right'].commands)
        self.assertEqual(list(range(1, len(sequences) + 1)), sorted(sequences))
        self.assertEqual(1, self.control.calls.count('enter_external'))
        self.assertEqual(1, self.control.calls.count('exit_external'))
        self.assertEqual(0, self.control.command_count)


if __name__=='__main__':
    unittest.main(verbosity=2)
