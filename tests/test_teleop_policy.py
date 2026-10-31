"""Synthetic offline tests. These are not robot-captured golden evidence."""
import dataclasses
import json
import math
from pathlib import Path
import signal
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from examples.teleop_policy.teleop_policy import JoyCommand, Policy, bodycmd, live, objects


ROOT = Path(__file__).parent / "fixtures/teleop"
METADATA = json.loads((ROOT/'model-metadata.json').read_text())['metadata']
CONTRACT = json.loads((ROOT/'action17_contract.json').read_text())
NAMES = (
    'left_hip_pitch_joint','left_hip_roll_joint','left_hip_yaw_joint',
    'left_knee_joint','left_ankle_pitch_joint','left_ankle_roll_joint',
    'right_hip_pitch_joint','right_hip_roll_joint','right_hip_yaw_joint',
    'right_knee_joint','right_ankle_pitch_joint','right_ankle_roll_joint',
    'waist_yaw_joint','waist_pitch_joint','waist_roll_joint',
    'left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint',
    'left_elbow_joint','left_wrist_roll_joint','left_wrist_yaw_joint','left_wrist_pitch_joint',
    'right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint',
    'right_elbow_joint','right_wrist_roll_joint','right_wrist_yaw_joint','right_wrist_pitch_joint',
    'neck_pitch_joint','neck_yaw_joint',
)


def fixture():
    body = NS(slot_count=31,joint_names=NAMES,
              joints=tuple(NS(body_index=i, is_mock=n=='waist_roll_joint', supported_fields=31,
                              valid_limits=31, q_min=-3., q_max=3.) for i,n in enumerate(NAMES)))
    motors = tuple(NS(error_code=0,
                      q=float(i),dq=float(100+i)) for i,n in enumerate(NAMES))
    # TRUNK first and deliberately different, preventing accidental imu_states[0].
    trunk = NS(location='TRUNK',state=NS(quaternion=(0,1,0,0),gyroscope=(9,9,9)),status=1)
    hip = NS(location='HIP',state=NS(quaternion=(1,0,0,0),gyroscope=(1,2,3)),status=1)
    state = NS(joints=motors,imu_states=(trunk,hip),attitude_valid=True,
               state_id=987654321,ts_state_real_us=1792000000123456)
    return body,state


API = NS(BodyJointCmd=dataclasses.make_dataclass('BodyJointCmd',[(k,float) for k in ('q','dq','tau','kp','kd')]),
         BodyCmd=lambda **kwargs:NS(**kwargs),BodyCmdAction=NS(EXTERNAL='EXTERNAL_SYMBOLIC'),
         BodyImuLocation=NS(HIP='HIP'),ExternalPhase=NS(RUNNING='RUNNING',IDLE='IDLE'),
         ServiceStatus=NS(SUCCESS='SUCCESS_SYMBOLIC',PARTIAL_SUCCESS='PARTIAL_SUCCESS_SYMBOLIC',
                          UNAVAILABLE='UNAVAILABLE_SYMBOLIC',INTERNAL_ERROR='INTERNAL_ERROR_SYMBOLIC'))


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.policy = Policy(METADATA,CONTRACT)
        self.body,self.state = fixture()

    def test_required_leg_and_upper_mock_are_rejected_before_target_selection(self):
        self.assertEqual(len(self.policy.names), 30)
        self.assertTrue(self.body.joints[14].is_mock)
        self.assertEqual(len(self.policy.roster(self.body)), 31)
        for name in (self.policy.legs[0], self.policy.upper[-1]):
            with self.subTest(name=name):
                joints = list(self.body.joints)
                i = self.body.joint_names.index(name)
                joints[i] = NS(body_index=i, is_mock=True, supported_fields=31, valid_limits=31,
                               q_min=-3., q_max=3.)
                changed = NS(slot_count=31, joint_names=self.body.joint_names, joints=tuple(joints))
                with self.assertRaisesRegex(ValueError, 'unavailable'):
                    self.policy.roster(changed)

    def test_named_full_frame_and_initial_backfill(self):
        obs = self.policy.observation(self.state,self.body,(.25,-.5,.125,.75),'HIP')
        # Manually frozen expected axis: wrists pitch/yaw and neck yaw/pitch swap.
        order = list(range(12))+[12,13,15,16,17,18,19,21,20,22,23,24,25,26,28,27,30,29]
        default_legs = [-.2,0,0,.4,-.2,0,-.2,0,0,.4,-.2,0]
        import struct
        raw = [.25,-.5,.125,.75,1,2,3,0,0,-1]
        raw += [i-(default_legs[j] if j<12 else 0) for j,i in enumerate(order)]
        raw += [100+i for i in order]+[0]*12
        expected = struct.unpack('82f',struct.pack('82f',*raw))
        self.assertEqual(obs,expected*6)

    def test_array_input_matches_dto_without_materializing_joints_or_imu(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest('NumPy is optional for source-only tests')
        from lowlevel.BodyImuLocation import BodyImuLocation
        class Arrays:
            @property
            def joints(self):
                raise AssertionError('native policy must not construct joint DTOs')
            @property
            def imu_states(self):
                raise AssertionError('native policy must not construct IMU DTOs')
        arrays = Arrays()
        arrays.q = np.asarray([j.q for j in self.state.joints], dtype=np.float32)
        arrays.dq = np.asarray([j.dq for j in self.state.joints], dtype=np.float32)
        arrays.error_code = np.zeros(31, dtype=np.uint32)
        arrays.imu_values = np.zeros((2, 13), dtype=np.float32)
        arrays.imu_values[1, :7] = [1, 0, 0, 0, 1, 2, 3]
        arrays.imu_location = [BodyImuLocation.TRUNK, BodyImuLocation.HIP]
        arrays.imu_status = [1, 1]
        arrays.imu_state_present = [1, 1]
        arrays.attitude_valid = True
        command = (.25, -.5, .125, .75)
        expected = self.policy.observation(self.state, self.body, command, 'HIP')
        actual = Policy(METADATA, CONTRACT).observation(arrays, self.body, command, BodyImuLocation.HIP)
        self.assertEqual(actual, expected)

    def test_oldest_to_newest_and_action_feedback(self):
        self.policy.observation(self.state,self.body,(0,0,0,.75),'HIP')
        self.policy.targets([.125]*12,self.body)
        obs = self.policy.observation(self.state,self.body,(.5,0,0,.75),'HIP')
        self.assertEqual(obs[0],0)
        self.assertEqual(obs[-82],.5)
        self.assertEqual(obs[-12:],(.125,)*12)

    def test_projected_gravity_pitch_90_degrees(self):
        self.state.imu_states[1].state.quaternion=(math.sqrt(.5),0,math.sqrt(.5),0)
        obs=self.policy.observation(self.state,self.body,(0,0,0,.75),'HIP')
        self.assertAlmostEqual(obs[7],1)
        self.assertAlmostEqual(obs[8],0)
        self.assertAlmostEqual(obs[9],0)

    def test_no_scale_applied_twice(self):
        md=dict(METADATA,obs_scale_base_ang_vel='100')
        obs=Policy(md,CONTRACT).observation(self.state,self.body,(0,0,0,.75),'HIP')
        self.assertEqual(obs[4:7],(1,2,3))

    def test_invalid_hip_does_not_fall_back_to_trunk(self):
        self.state.imu_states[1].status=2
        with self.assertRaises(ValueError):
            self.policy.observation(self.state,self.body,(0,0,0,.75),'HIP')

    def test_missing_joint_or_nan_rejected_but_roll_not_read(self):
        self.state.joints[14].q=float('nan')
        self.policy.observation(self.state,self.body,(0,0,0,.75),'HIP')
        self.state.joints[20].error_code=20
        with self.assertRaises(ValueError):
            self.policy.observation(self.state,self.body,(0,0,0,.75),'HIP')

    def test_duplicate_or_unknown_roster_rejected(self):
        self.body.joint_names=NAMES[:-1]+(NAMES[0],)
        with self.assertRaises(ValueError):self.policy.roster(self.body)

    def test_31_output_metadata_decode_pd_and_roll_placeholder(self):
        targets=self.policy.targets([1]*12,self.body)
        self.assertEqual(len(targets),31)
        self.assertAlmostEqual(targets[0].q,.05)
        self.assertEqual(targets[0].kp,176.9)
        self.assertEqual(targets[12].kp,150)
        self.assertEqual(targets[21].kp,80)
        self.assertEqual(targets[29].kp,20)
        self.assertEqual(dataclasses.asdict(targets[14]),dict(q=0,dq=0,tau=0,kp=0,kd=0))

    def test_no_new_fallback_limit_from_joint_target_metadata(self):
        self.assertIsNone(self.policy.limits)
        with self.assertRaisesRegex(ValueError, 'outside BodyInfo limit'):
            self.policy.targets([100]*12, self.body)
        joints = tuple(NS(**(dict(vars(cap), q_min=-100., q_max=100.)
                              if name in self.policy.legs else vars(cap)))
                       for name, cap in zip(self.body.joint_names, self.body.joints))
        wide = NS(slot_count=31, joint_names=self.body.joint_names, joints=joints)
        self.assertAlmostEqual(self.policy.targets([100]*12, wide)[0].q,24.8)

    def test_declared_clipping_and_leg_limits(self):
        md=dict(METADATA,clip_actions='2',joint_pos_limit_lower=','.join(['-.1']*12),joint_pos_limit_upper=','.join(['.1']*12))
        p=Policy(md,CONTRACT)
        self.assertEqual(p.targets([100]*12,self.body)[0].q,.1)
        self.assertEqual(p.previous_actions,(2,)*12)

    def test_wrong_export_dimensions_rejected(self):
        with self.assertRaises(ValueError):Policy(dict(METADATA,frame_dim='66'),CONTRACT)

    def test_bodycmd_explicit_action_and_actual_input_causality(self):
        cmd=bodycmd(API,self.policy.targets([0]*12,self.body),self.state,)
        self.assertEqual(cmd.action,'EXTERNAL_SYMBOLIC')
        self.assertEqual(cmd.input_state_id,987654321)
        self.assertEqual(cmd.input_state_ts_real_us,1792000000123456)
        self.assertEqual(len(cmd.joints),31)
        self.assertFalse(hasattr(cmd.joints[0],'mode'))


class JoyTests(unittest.TestCase):
    def test_profile_mapping_slew_height_disabled_axis(self):
        p=JoyCommand(CONTRACT)
        self.assertEqual(p.step((0,0.049,0,0)),(0,0,0,.855))
        first=p.step((0,1,0,1))
        self.assertAlmostEqual(first[0],.02)
        self.assertAlmostEqual(first[2],.026)
        for _ in range(200):last=p.step((1,-1,0,-1))
        self.assertAlmostEqual(last[0],-.4)
        self.assertAlmostEqual(last[1],.5)
        self.assertAlmostEqual(last[2],-.8)
        self.assertEqual(last[3],.855)


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        # These lifecycle cases use symbolic service DTOs; formatting is tested
        # separately with a real public Info DTO.
        formatter = patch('locomotion_aorta._policy_state.format_robot_info', return_value='Info')
        formatter.start()
        self.addCleanup(formatter.stop)

    def run_case(self,mode,reply_status=API.ServiceStatus.PARTIAL_SUCCESS):
        body,state=fixture()
        # Ordinary synthetic stand values; no robot-captured evidence claimed.
        for m in state.joints:m.q=0;m.dq=0
        stop=threading.Event();events=[]
        status=NS(phase='RUNNING',current_action='EXTERNAL',generation=3,
                  client_id=9,control_id=13,reject_count=0)
        class Controller:
            def get_robot_info(self,timeout):return NS(info=NS(body=body),status=reply_status,error_message='hands unavailable')
            def wait_body_state(self,timeout):
                if 'enter' in events and mode=='wait_error':raise TimeoutError('actual feedback timeout')
                state.state_id += 1
                return state
            wait_state = wait_body_state
            def enter_external(self,timeout):
                events.append('enter')
                if mode=='entry_error':raise TimeoutError('entry interrupted after PREPARE')
                return status
            def publish(self,cmd, *, target_names, required_fields):
                if mode=='sdk_guard':raise ValueError('existing SDK command/capability guard')
                events.append('publish');stop.set()
            def exit_external(self,timeout):
                events.append('cancel')
                return NS(current_action='WRONG' if mode=='bad_exit' else 'PASSIVE',phase='IDLE')
        controller=Controller();controller.status=status
        class Model:
            def infer(self,obs):
                if mode in ('sigint','sigterm'):
                    # Deliver a real Python signal during inference, before publish.
                    signal.raise_signal(signal.SIGINT if mode=='sigint' else signal.SIGTERM)
                return [0]*12
        old={sig:signal.signal(sig,lambda n,f:stop.set()) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            live(controller,API,Policy(METADATA,CONTRACT),Model(),lambda:(),stop,5,sleep=lambda _:None)
        finally:
            for sig,handler in old.items():signal.signal(sig,handler)
            self.assertEqual(events[-1],'cancel')
            if mode in ('sigint','sigterm','entry_error','wait_error'):self.assertNotIn('publish',events)
        return events

    def published_state_ids(self, frame_count):
        import locomotion_aorta as api
        from locomotion_aorta.controller import _validate_command
        body, state = fixture()
        stop = threading.Event()
        commands, events = [], []
        status = NS(phase=api.ExternalPhase.RUNNING, current_action='EXTERNAL',
                    generation=3, client_id=9, control_id=13, reject_count=0)
        targets = tuple(api.BodyJointCmd() for _ in range(31))

        class Controller:
            def get_robot_info(self, timeout):
                return NS(status=api.ServiceStatus.SUCCESS, info=NS(body=body))
            def wait_body_state(self, timeout):
                state.state_id += 1
                return state
            wait_state = wait_body_state
            def enter_external(self, timeout):
                events.append('enter')
                return status
            def publish(self, command, *, target_names, required_fields):
                if target_names != tuple(n for n in body.joint_names if n != 'waist_roll_joint') or required_fields != 0x19:
                    raise AssertionError('policy must declare its complete physical target scope and PD fields')
                # Exercise the real controller guard at startup and both sides
                # of the uint16 boundary, without transport or a real model.
                if len(commands) in (0, 65534, 65535, 65536):
                    _validate_command(command)
                commands.append(command.input_state_id)
                if len(commands) == frame_count:
                    stop.set()
            def exit_external(self, timeout):
                events.append('cancel')
                return NS(current_action='PASSIVE')

        class PolicyStub:
            names = tuple(n for n in body.joint_names if n != 'waist_roll_joint')
            contract = CONTRACT
            period = 0
            history = []
            def roster(self, body):
                return {}
            def observation(self, *args):
                return ()
            def targets(self, actions, body):
                return targets

        controller = Controller()
        controller.status = status
        live(controller, api, PolicyStub(), NS(infer=lambda obs: ()), lambda: (),
             stop, 5, sleep=lambda seconds: None, monotonic=lambda: 0)
        self.assertEqual(events, ['enter', 'cancel'])
        return commands

    def test_live_first_command_uses_fresh_input_identity_without_cmd_id(self):
        self.assertEqual(self.published_state_ids(1), [987654323])

    def test_live_input_identity_does_not_wrap_at_uint16(self):
        ids = self.published_state_ids(65537)
        self.assertEqual(len(set(ids)), 65537)
        self.assertTrue(all(b-a == 1 for a, b in zip(ids, ids[1:])))

    def fresh_state_case(self, ids, *, stop_during_wait=False):
        from test_examples import FakeClock
        body, state = fixture()
        clock, stop = FakeClock(), threading.Event()
        published, inferred, reads, events = [], [], [], []
        status = NS(phase='RUNNING', current_action='EXTERNAL', generation=3,
                    client_id=9, control_id=13, reject_count=0)
        class Controller:
            def get_robot_info(self, timeout):
                return NS(info=NS(body=body), status=API.ServiceStatus.SUCCESS, error_message='')
            def wait_body_state(self, timeout):
                if len(reads) >= 1100:
                    raise AssertionError('test read budget exceeded: frozen BodyState loop did not terminate')
                reads.append(timeout)
                state.state_id = ids[min(len(reads)-1, len(ids)-1)]
                if stop_during_wait and len(reads) == 3:
                    stop.set()
                return state
            wait_state = wait_body_state
            def enter_external(self, timeout):
                events.append('enter')
                return status
            def publish(self, command, *, target_names, required_fields):
                published.append(command)
                if command.input_state_id == ids[-1] and len(set(ids)) > 2:
                    stop.set()
            def exit_external(self, timeout):
                events.append('cancel')
                return NS(current_action='PASSIVE')
        controller = Controller()
        controller.status = status
        model = NS(infer=lambda obs: inferred.append(obs) or [0]*12)
        try:
            live(controller, API, Policy(METADATA, CONTRACT), model, lambda: (), stop,
                 timeout=1, sleep=clock.sleep, monotonic=clock.monotonic)
        finally:
            self.assertEqual(events, ['enter', 'cancel'])
            self.assertEqual(len(inferred), len(published))
            if len(set(ids)) == 1:
                self.assertEqual(inferred, [])
                self.assertEqual(published, [])
            self.assertTrue(all(0 < remaining <= 1 for remaining in reads))
        return published, inferred, clock

    def test_body_state_frozen_before_first_inference_times_out_and_cancels(self):
        # No fresh input after the preflight sample: no new leg targets at all.
        with self.assertRaisesRegex(TimeoutError, 'BodyState did not advance'):
            self.fresh_state_case([10, 10])

    def test_body_state_stops_or_regresses_after_publish_no_more_inference_then_cancel(self):
        for tail in (11, 10):
            with self.subTest(tail=tail):
                # Capture the first successful inference/publication; all later
                # cached or out-of-order samples must be held, then CANCEL.
                body, state = fixture()
                from test_examples import FakeClock
                clock, stop = FakeClock(), threading.Event()
                outputs, inferences, events = [], [], []
                status = NS(phase='RUNNING', current_action='EXTERNAL', generation=3,
                            client_id=9, control_id=13, reject_count=0)
                class Controller:
                    reads = 0
                    def get_robot_info(self, timeout):
                        return NS(info=NS(body=body), status=API.ServiceStatus.SUCCESS, error_message='')
                    def wait_body_state(self, timeout):
                        self.reads += 1
                        if self.reads > 1100:
                            raise AssertionError('test read budget exceeded: frozen BodyState loop did not terminate')
                        state.state_id = 10 if self.reads == 1 else 11 if self.reads == 2 else tail
                        return state
                    wait_state = wait_body_state
                    def enter_external(self, timeout):
                        events.append('enter')
                        return status
                    def publish(self, command, *, target_names, required_fields):
                        outputs.append(command)
                    def exit_external(self, timeout):
                        events.append('cancel')
                        return NS(current_action='PASSIVE')
                controller = Controller()
                controller.status = status
                with self.assertRaisesRegex(TimeoutError, 'BodyState did not advance'):
                    live(controller, API, Policy(METADATA, CONTRACT),
                         NS(infer=lambda obs: inferences.append(obs) or [0]*12), lambda: (), stop, 1,
                         sleep=clock.sleep, monotonic=clock.monotonic)
                self.assertEqual([cmd.input_state_id for cmd in outputs], [11])
                self.assertEqual(len(inferences), 1)
                self.assertEqual(events, ['enter', 'cancel'])

    def test_cached_or_regressed_samples_are_waited_out_before_fresh_inference(self):
        outputs, inferred, clock = self.fresh_state_case([10, 10, 9, 10, 11, 11, 12])
        self.assertEqual([cmd.input_state_id for cmd in outputs], [11, 12])
        self.assertEqual(len(inferred), 2)
        self.assertGreater(clock.monotonic(), 0)

    def test_stop_while_waiting_for_update_cancels_without_new_targets(self):
        outputs, inferred, _ = self.fresh_state_case([10, 10], stop_during_wait=True)
        self.assertEqual(outputs, [])
        self.assertEqual(inferred, [])

    def test_normal_exit_cancels(self):self.assertEqual(self.run_case('normal'),['enter','publish','cancel'])
    def test_success_service_status_runs_and_cancels(self):
        self.assertEqual(self.run_case('normal',API.ServiceStatus.SUCCESS),['enter','publish','cancel'])

    def test_service_failure_with_nonempty_body_preserves_status_and_message(self):
        body,_=fixture()
        for status in (API.ServiceStatus.UNAVAILABLE,API.ServiceStatus.INTERNAL_ERROR,'UNKNOWN_STATUS'):
            with self.subTest(status=status):
                events=[]
                class Controller:
                    def get_robot_info(self,timeout):
                        events.append('info')
                        return NS(status=status,info=NS(body=body),error_message='discovery failed exactly')
                    def wait_body_state(self,timeout):
                        raise AssertionError('must not wait for feedback on failed discovery')
                    wait_state = wait_body_state
                with self.assertRaises(RuntimeError) as caught:
                    live(Controller(),API,Policy(METADATA,CONTRACT),None,lambda:(),threading.Event(),5)
                self.assertIn(repr(status),str(caught.exception))
                self.assertIn('discovery failed exactly',str(caught.exception))
                self.assertEqual(events,['info'])

    def test_accepted_service_status_still_requires_real_body(self):
        for status in (API.ServiceStatus.SUCCESS,API.ServiceStatus.PARTIAL_SUCCESS):
            for info in (None,NS(body=None)):
                with self.subTest(status=status,info=info):
                    class Controller:
                        def get_robot_info(self,timeout):
                            return NS(status=status,info=info,error_message='body missing')
                    with self.assertRaisesRegex(RuntimeError,'real body capabilities'):
                        live(Controller(),API,Policy(METADATA,CONTRACT),None,lambda:(),threading.Event(),5)

    def test_sigint_stops_targets_then_cancels(self):self.run_case('sigint')
    def test_sigterm_stops_targets_then_cancels(self):self.run_case('sigterm')
    def test_entry_failure_still_cancels(self):
        with self.assertRaises(TimeoutError):self.run_case('entry_error')
    def test_feedback_failure_cancels_without_retry(self):
        with self.assertRaises(TimeoutError):self.run_case('wait_error')
    def test_ack_without_passive_not_reported_as_complete(self):
        with self.assertRaises(RuntimeError):self.run_case('bad_exit')
    def test_sdk_rejection_propagates_and_cancels(self):
        with self.assertRaisesRegex(ValueError,'existing SDK'):
            self.run_case('sdk_guard')


if __name__=='__main__':unittest.main()


class ActualSdkDtoIntegration(unittest.TestCase):
    def test_policy_accepts_typed_body_and_encodes_explicit_generated_external(self):
        import locomotion_aorta as api
        from locomotion_aorta.transport import _fill_body_cmd
        import flatbuffers
        import lowlevel.BodyCmd as command_module
        body, state = fixture()
        capability = tuple(api.BodyJointCapability(i,api.BodySlotRole.ACTIVE,j.is_mock,
                              j.supported_fields,j.valid_limits,-3.,3.,0.,0.,0.,0.,0.,0.,0.,0.)
                           for i,j in enumerate(body.joints))
        typed_body = api.BodyInfo(0,31,body.joint_names,capability,
                                 tuple(api.BodyMotorIdentity('synthetic',i+1,i != 14) for i in range(31)))
        motors = tuple(api.BodyJointState(q=m.q,dq=m.dq) for m in state.joints)
        imus = tuple(api.BodyImuSample(getattr(api.BodyImuLocation,m.location),
                         api.BodyImuState(m.state.quaternion,m.state.gyroscope,(0.,0.,9.81),(0.,0.,0.)),
                         m.status) for m in state.imu_states)
        typed_state = api.BodyState(state.state_id,motors,runtime_id=11,ts_state_real_us=state.ts_state_real_us,
                                   attitude_valid=True,imu_states=imus)
        policy = Policy(METADATA,CONTRACT)
        observation = policy.observation(typed_state,typed_body,(0.,0.,0.,.85),api.BodyImuLocation.HIP)
        self.assertEqual(len(observation),492)
        targets = policy.targets((0.,)*12,typed_body)
        command = dataclasses.replace(bodycmd(api,targets,typed_state),
                                      runtime_id=11, control_id=13, session_id=3, sequence=1)
        b=flatbuffers.Builder(1);b.Finish(_fill_body_cmd(NS(body_cmd_module=command_module),command,b,0))
        view=command_module.BodyCmd.GetRootAs(b.Output())
        self.assertEqual(view.Action(),api.BodyCmdAction.EXTERNAL)
        self.assertNotEqual(view._tab.Offset(22),0)
        self.assertEqual(view.JointsLength(),31)
        self.assertEqual(view.Joints(14).Q(),0.)
        self.assertEqual(view.InputStateId(),typed_state.state_id)
        self.assertEqual(view.InputStateTsRealUs(),typed_state.ts_state_real_us)
