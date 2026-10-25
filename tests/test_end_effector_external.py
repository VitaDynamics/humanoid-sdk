"""Single CLI entry with synthetic endpoints and the original Body lifecycle.

Uses real new DTO/codec fields and the shared controller allocator. This is
regression source, not measured robot output or proof of transport delivery.
"""
from dataclasses import replace
import io
import math
import signal
import unittest
from unittest.mock import Mock, patch

from examples.end_effector import end_effector_external as demo
from locomotion_aorta import BODY_CMD_TOPIC
from locomotion_aorta import _end_effector_codec as codec
from locomotion_aorta.end_effector_types import (
    CommandKey, ControlRef, DescriptorRef, DeviceModel, EndEffectorSubmission,
    FeedbackAppliedTarget, Field, InhibitReason, PositionDomain, SampleStatus,
    SampleStream,
)
from test_end_effector_codec import decode, info_object, state_object, touch_object
from test_examples import FakeClock, RecordingController


class Endpoint:
    """Deterministic sensor/admission seam, not another permission authority."""
    def __init__(self, connection, side='right', *, hand=False, echo=True,
                 stale=False, reject=False, inhibited=False, touch=False):
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
        self.echo, self.stale, self.reject, self.inhibited = echo, stale, reject, inhibited
        self.after_publish = None

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

    def wait_info(self, timeout):
        return self.info

    def wait_state(self, timeout, **kwargs):
        return self.state

    def publish(self, target, *, input_observations):
        codec.validate_observations(self.info.descriptor_ref, input_observations, sources=self.observation_sources)
        prepared = codec.prepare_target(self.info, target)
        context = self.connection.controller._reserve_command()
        submission = EndEffectorSubmission(self.info.descriptor_ref, self.info.layout_id, self.info.side,
            ControlRef(context.client_id, context.lease_id), CommandKey(context.session_id, context.sequence),
            prepared, input_observations=input_observations)
        self.commands.append(submission)
        self.times.append(self.connection.clock.monotonic())
        self.observations.append(input_observations)
        changes = dict(received_key=submission.key, key_valid_mask=1)
        if self.echo and not self.reject:
            changes.update(host_target_key=submission.key, key_valid_mask=3)
        if self.reject:
            changes.update(reject_count=1, diagnostic_valid_mask=4)
        self._state = replace(self._state, **changes)
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
            keys.extend(cmd.key for cmd in client.commands)
            for cmd in client.commands:
                self.assertEqual(cmd.control_ref, ControlRef('example-client', 'lease-example'))
                self.assertEqual(cmd.target.pos, (.0625,))
                self.assertEqual(cmd.target.vel_limit, (0.,))
                self.assertFalse(cmd.target.command_fields[0] & Field.TAU)
                self.assertEqual(cmd.input_observations[0].descriptor_ref, client.info.descriptor_ref)
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
        c.get_robot_info = Mock(side_effect=AssertionError('Body discovery'))
        e = Endpoint(c)
        self.run_case(c,[e],body_enabled=False)
        self.assertTrue(e.commands)
        self.assertEqual(c.command_count,0)
        c.wait_state.assert_not_called(); c.get_robot_info.assert_not_called()
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_mixed_rates_do_not_couple_endpoints_or_speed_up_body(self):
        c = self.controller()
        clients = [Endpoint(c,side) for side in ('left','right')]
        self.run_case(c,clients,duration=.4,rates={'left':20.,'right':100.})
        self.assertGreater(len(clients[1].commands),len(clients[0].commands)*3)
        for times,period in ((c.command_times,.02),(clients[0].times,.05),(clients[1].times,.01)):
            self.assertTrue(all(b-a >= period-1e-10 for a,b in zip(times,times[1:])))
        self.assertEqual(c.calls.count('enter_external'),1)

    def test_slow_host_never_replays_missed_deadlines(self):
        original_sleep = self.clock.sleep
        self.clock.sleep = lambda seconds: original_sleep(seconds+.031)
        c = self.controller(); e=Endpoint(c)
        self.run_case(c,[e],duration=.2,rates={'right':100.})
        self.assertTrue(all(b-a >= .031 for a,b in zip(e.times,e.times[1:])))
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_explicit_units_and_none_zero_signed_torque_remain_distinct(self):
        c = self.controller(); e=Endpoint(c)
        for torque in (None,(0.,),(-.25,)):
            with self.subTest(torque=torque):
                target=self.target(e,torque=torque)
                self.assertEqual(bool(target.command_fields[0]&Field.TAU),torque is not None)
                self.assertEqual(target.tau_cmd,(0.,) if torque is None else torque)
        with self.assertRaisesRegex(ValueError,'unit'):
            demo.position_target(e.info,(.05,),unit='rad')
        with self.assertRaises(ValueError): self.target(e,vel_limit=(-.1,))
        with self.assertRaises(ValueError): self.target(e,torque=(2.,))

    def test_unknown_faulted_nonfinite_wrong_ref_and_bad_target_fail_before_prepare(self):
        for changes,target_options in (({'sample_status':SampleStatus.NO_SNAPSHOT},{}),
                ({'valid_fields':(0,)},{}),({'pos':(math.nan,)},{}),
                ({'descriptor_ref':DescriptorRef(100,2)},{}),
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

    def test_reject_inhibit_stale_and_received_only_abort_once(self):
        for options in ({'reject':True},{'inhibited':True},{'stale':True},{'echo':False}):
            with self.subTest(options=options):
                c=self.controller(); e=Endpoint(c,**options)
                with self.assertRaises((RuntimeError,TimeoutError)):
                    self.run_case(c,[e],duration=.4)
                self.assertEqual(c.calls.count('enter_external'),1)
                self.assertEqual(c.calls.count('exit_external'),1)
                if options.get('echo') is False:
                    self.assertTrue(e._state.key_valid_mask&1)
                    self.assertFalse(e._state.key_valid_mask&2)

    def test_descriptor_change_after_first_send_stops_and_cancels(self):
        c=self.controller(); e=Endpoint(c)
        e.after_publish=lambda _: setattr(e,'_state',replace(e._state,descriptor_ref=DescriptorRef(999,1)))
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
        self.assertEqual(e.commands[0].target.pos,(.0625,))
        self.assertEqual(c.calls.count('exit_external'),1)

    def test_unconfirmed_cancel_never_reports_passive_success(self):
        c=self.controller(cancel_action='FIXED_STAND'); e=Endpoint(c)
        with self.assertRaisesRegex(RuntimeError,'expected PASSIVE'): self.run_case(c,[e])
        self.assertTrue(any('CANCEL UNCONFIRMED' in line for line in self.events))
        self.assertFalse(any('CANCEL confirmed' in line for line in self.events))

    def test_new_state_has_no_numeric_host_target_for_duration_or_arrival(self):
        c=self.controller(); e=Endpoint(c)
        e._state=replace(e._state,feedback_applied_target=FeedbackAppliedTarget((.12,),(.9,),(5,)))
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

    def test_feedback_preserves_distinct_keys_validity_raw_and_incomplete_progress(self):
        c=self.controller(); e=Endpoint(c)
        e._state=replace(e._state,received_key=CommandKey(17,9),host_target_key=CommandKey(17,8),
            key_valid_mask=3,pos=(math.nan,),valid_fields=(0,))
        demo.print_feedback(e,self.events.append)
        text='\n'.join(self.events)
        self.assertIn('sequence=9',text); self.assertIn('sequence=8',text)
        self.assertIn('pos=(nan,)',text); self.assertIn('valid=(0,)',text)
        self.assertIn('progress_complete=False',text); self.assertIn('raw_pos=(-32768,)',text)

    def test_cli_default_and_dry_run_borrow_readonly_source_without_controller(self):
        for flags in ([],['--dry-run']):
            source,connection,client=Mock(),Mock(),Mock()
            source.return_value.__enter__=Mock(return_value=connection)
            source.return_value.__exit__=Mock(return_value=False)
            client.return_value.__enter__=Mock(return_value=Mock(side='right'))
            client.return_value.__exit__=Mock(return_value=False)
            controller=Mock()
            with patch.object(demo,'observe') as observe:
                demo.main(['--side','right']+flags,_state_source_factory=source,
                          _client_factory=client,_controller_factory=controller)
            controller.assert_not_called(); observe.assert_called_once()
            client.assert_called_once_with('right',connection=connection,read_only=True,observation_sources=())

    def test_cli_none_does_not_construct_an_endpoint_or_request_control(self):
        source,connection,client,controller=Mock(),Mock(),Mock(),Mock()
        source.return_value.__enter__=Mock(return_value=connection)
        source.return_value.__exit__=Mock(return_value=False)
        with patch('builtins.print'):
            demo.main(['--side','none'],_state_source_factory=source,
                      _client_factory=client,_controller_factory=controller)
        client.assert_not_called(); controller.assert_not_called()
        connection.wait_state.assert_called_once_with(5.0)

    def test_cli_bad_options_fail_before_any_factory_or_rpc(self):
        cases=(['--ee-only','--side','none'],['--dry-run','--touch','on'],
               ['--confirm-motion','--side','right'],['--confirm-motion','--dry-run'],
               ['--right-vel-limit','-1'],['--duration','nan'],['--loop'],
               ['--side','left','--pattern','sine'],['--acceleration-limit-m-s2','1'])
        for flags in cases:
            with self.subTest(flags=flags),patch('sys.stderr',new_callable=io.StringIO):
                source,client,controller=Mock(),Mock(),Mock()
                with self.assertRaises(SystemExit):
                    demo.main(flags,_state_source_factory=source,_client_factory=client,
                              _controller_factory=controller)
                source.assert_not_called(); client.assert_not_called(); controller.assert_not_called()


    def test_actual_client_publish_writes_new_root_K_and_same_body_allocator(self):
        from locomotion_aorta.end_effector import EndEffectorClient
        from test_end_effector_client import Node
        recorded=self.controller()
        controller=recorded.controller
        node=Node(); recorded.transport.node=node
        clients=[]
        for side,wire_side in (('left',1),('right',2)):
            client=EndEffectorClient(side,connection=controller,
                observation_sources=((SampleStream.EE_STATE,wire_side),))
            self.addCleanup(client.close); clients.append(client)
            info=info_object(side=wire_side); info.deviceModel=DeviceModel.AGX
            node.emit(side,'info',info); node.emit(side,'state',state_object(side=wire_side))
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
                state.receivedKey=node.bindings.CommandKey.CommandKeyT(wire.Key().SessionId(),wire.Key().Sequence())
                state.hostTargetKey=state.receivedKey; state.keyValidMask=3
                node.emit(side,'state',state)
            result.publish_typed=publish
            return result
        node.create_publisher_typed=publisher
        enter,exit_=controller.enter_external,controller.exit_external
        with patch.object(controller,'wait_state',recorded.wait_state,create=True), \
             patch.object(controller,'enter_external',wraps=enter) as entered, \
             patch.object(controller,'exit_external',wraps=exit_) as cancelled:
            demo.run(controller,clients,{c.side:demo.position_target(c.info,(.03125,),unit='m') for c in clients},
                duration=.08,timeout=.3,sleep=self.clock.sleep,monotonic=self.clock.monotonic,emit=self.events.append)
        entered.assert_called_once(); cancelled.assert_called_once()
        keys=[(value.session_id,value.sequence) for topic,value in recorded.transport.published if topic==BODY_CMD_TOPIC]
        for topic,wire in node.commands:
            self.assertIn('/end_effector/',topic)
            keys.append((wire.Key().SessionId(),wire.Key().Sequence()))
            self.assertEqual(wire.InputObservationsLength(),1)
            sample=wire.InputObservations(0)
            self.assertEqual((sample.Stream(),sample.Side()),(SampleStream.EE_STATE,wire.Side()))
            self.assertEqual(sample.DescriptorRef().RobotRuntimeId(),100)
            self.assertEqual(wire.ControlRef().LeaseId(),b'lease-example')
            self.assertEqual(wire.ProtocolVersion(),1)
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


class TouchObservationTest(unittest.TestCase):
    def make_client(self):
        c=Mock()
        c.state=decode('state',state_object())
        c.touch=decode('touch',touch_object())
        return c

    def test_rpc_success_waits_new_valid_state_then_real_touch_not_info(self):
        c=self.make_client(); initial=c.state; old=c.touch
        c.wait_state.side_effect=[replace(initial,state_id=2,status_valid_mask=7,touch_enabled=True),
                                 replace(initial,state_id=3,touch_enabled=True)]
        c.wait_touch.return_value=replace(old,touch_state_id=2)
        state,touch=demo.set_touch_and_observe(c,True,1.)
        c.set_touch.assert_called_once_with(True,timeout=1.)
        self.assertEqual(state.state_id,3); self.assertEqual(touch.touch_state_id,2)
        self.assertEqual(c.wait_state.call_args_list[0].kwargs['after_sample'],initial.sample_ref)
        self.assertEqual(c.wait_touch.call_args.kwargs['after_sample'],old.sample_ref)
        c.wait_info.assert_not_called()

    def test_off_requires_state_confirmation_but_not_an_impossible_new_touch(self):
        c=self.make_client(); c.wait_state.return_value=replace(c.state,state_id=2,touch_enabled=False)
        _,touch=demo.set_touch_and_observe(c,False,1.)
        self.assertIsNone(touch); c.wait_touch.assert_not_called()
        c.set_touch.assert_called_once()

    def test_wrong_touch_or_state_ref_and_rpc_failure_do_not_retry(self):
        for failure in ('state','touch','rpc'):
            c=self.make_client(); c.wait_state.return_value=replace(c.state,state_id=2,touch_enabled=True)
            c.wait_touch.return_value=replace(c.touch,touch_state_id=2)
            if failure=='state': c.wait_state.return_value=replace(c.wait_state.return_value,descriptor_ref=DescriptorRef(1,2))
            elif failure=='touch': c.wait_touch.return_value=replace(c.touch,descriptor_ref=DescriptorRef(1,2))
            else: c.set_touch.side_effect=OSError('RPC failure')
            with self.assertRaises((ValueError,OSError)): demo.set_touch_and_observe(c,True,1.)
            c.set_touch.assert_called_once()

    def test_rpc_consumes_the_same_budget_no_full_timeout_restart(self):
        clock=FakeClock(); c=self.make_client()
        c.set_touch.side_effect=lambda *_a,**_k:clock.sleep(.8)
        c.wait_state.return_value=replace(c.state,state_id=2,touch_enabled=True)
        c.wait_touch.side_effect=TimeoutError('no Touch')
        with self.assertRaises(TimeoutError): demo.set_touch_and_observe(c,True,1.,monotonic=clock.monotonic)
        self.assertAlmostEqual(c.wait_state.call_args.args[0],.2)
        self.assertAlmostEqual(c.wait_touch.call_args.args[0],.2)
        c.set_touch.assert_called_once()


if __name__=='__main__':
    unittest.main(verbosity=2)
