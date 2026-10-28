"""Real compiled Mock + SDK IPC on private loopback peers; no robot endpoints."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest


def client():
    from locomotion_aorta import (BodyCmd, BodyJointCmd, PullController,
                                  PullStateSource)
    group = os.environ['AORTA_GROUP']
    with PullController(group=group) as controller:
        source = PullStateSource(group=group)
        try:
            info = controller.get_robot_info(5)
            if info.info is None or info.info.body.slot_count != 31:
                raise AssertionError('31-slot RobotInfo missing')
            source_info = source.get_robot_info(5)
            if source_info.descriptor_ref != info.descriptor_ref:
                raise AssertionError('source and controller runtime differ')
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


def ee_client():
    from contextlib import ExitStack
    from locomotion_aorta import BodyCmd, BodyJointCmd, PullController
    from locomotion_aorta.end_effector import EndEffectorClient
    from locomotion_aorta.end_effector_types import ControlMode, EndEffectorTarget, Field, SampleStream, ServiceStatus
    with ExitStack() as stack:
        controller = stack.enter_context(PullController(group=os.environ['AORTA_GROUP']))
        roster = controller.get_robot_info(5)
        assert roster.info is not None and roster.info.body.slot_count == 31
        endpoints = [stack.enter_context(EndEffectorClient(side, connection=controller,
            observation_sources=((SampleStream.EE_STATE, number),)))
            for side, number in (('left',1), ('right',2))]
        infos = [endpoint.wait_info(5) for endpoint in endpoints]
        assert infos[0].descriptor_ref == infos[1].descriptor_ref
        assert roster.descriptor_ref == infos[0].descriptor_ref
        state = controller.wait_body_state(5)
        assert state.protocol_version == 3
        assert state.descriptor_ref == infos[0].descriptor_ref == infos[1].descriptor_ref
        assert not state.applied_key_valid
        controller.enter_external(5)
        try:
            def body_command():
                latest = controller.wait_body_state(1)
                return controller.publish(BodyCmd(joints=tuple(
                    BodyJointCmd() if i == 14 else BodyJointCmd(q=j.q, kp=1, kd=.5)
                    for i,j in enumerate(latest.joints)), input_state_id=latest.state_id,
                    input_state_ts_real_us=latest.ts_state_real_us))
            first = body_command()
            submitted = []
            for endpoint, info in zip(endpoints, infos):
                sample = endpoint.wait_state(5)
                n = len(info.axis_names)
                target = EndEffectorTarget((ControlMode.POSITION,)*n, (Field.POS,)*n, pos=(.03125,)*n)
                submitted.append(endpoint.publish(target, input_observations=(sample.sample_ref,)))
            assert [s.key.sequence for s in submitted] == [first.sequence+1, first.sequence+2]
            assert all(s.key.session_id == first.session_id for s in submitted)
            for endpoint, info in zip(endpoints, infos):
                deadline = time.monotonic()+2
                while True:
                    sample = endpoint.wait_state(max(.001, deadline-time.monotonic()))
                    if all(mask & Field.POS for mask in sample.valid_fields) and all(q == .03125 for q in sample.pos):
                        break
                    if time.monotonic() >= deadline: raise AssertionError('synthetic EE position not observed')
                    endpoint.wait_state(deadline-time.monotonic(), after_sample=sample.sample_ref)
                assert all(info.is_mock) and all(q == .03125 for q in sample.pos)
            left = endpoints[0]
            before = left.wait_state(1)
            reply = left.set_touch(True, timeout=1.)
            assert reply.status == ServiceStatus.SUCCESS and reply.descriptor_ref == infos[0].descriptor_ref
            deadline = time.monotonic()+2
            sample = before
            while not (sample.status_valid_mask & 8 and sample.touch_enabled):
                sample = left.wait_state(max(.001, deadline-time.monotonic()), after_sample=sample.sample_ref)
                if time.monotonic() >= deadline: raise AssertionError('typed touch did not update State')
            touch = left.wait_touch(2)
            assert touch.descriptor_ref == infos[0].descriptor_ref and touch.payload == b'\x80\xff'
            assert all(stamp.validity == 1 and stamp.ts_acquired_hw_us == 0 for stamp in touch.region_stamps)
            second = body_command()
            assert second.sequence == submitted[-1].key.sequence+1  # RPC consumed no motion key.
            deadline = time.monotonic()+2
            while True:
                state = controller.wait_body_state(1)
                if state.applied_key_valid and state.applied_sequence == second.sequence: break
                if time.monotonic() >= deadline: raise AssertionError('BodyState3 association missing')
                time.sleep(.005)
            assert state.applied_session_id == second.session_id
            assert state.ts_control_us > 0 and state.association_source_type == 1
            assert state.descriptor_ref == infos[0].descriptor_ref
            left.set_touch(False, timeout=1.)
            deadline = time.monotonic()+2
            while sample.touch_enabled:
                sample = left.wait_state(max(.001, deadline-time.monotonic()), after_sample=sample.sample_ref)
                if time.monotonic() >= deadline: raise AssertionError('disable not reflected in State')
        finally:
            status = controller.exit_external(5)
        assert status.current_action == 'PASSIVE'
        for endpoint in endpoints:
            deadline = time.monotonic()+2
            sample = endpoint.wait_state(1)
            while sample.output_allowed:
                sample = endpoint.wait_state(max(.001, deadline-time.monotonic()), after_sample=sample.sample_ref)
                if time.monotonic() >= deadline: raise AssertionError('EE permission survived CANCEL')


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
                '--client'])
            rc = run.run(args)
            logs = '\n'.join(p.read_text() for p in output.glob('*.log'))
            self.assertEqual(rc, 0, logs)
            result = json.loads((output / 'mock-result.json').read_text())
            self.assertEqual(result['endpoint_rc'], 0)
            self.assertEqual(result['parse_errors'], 0)
            self.assertEqual(result['accepted'], 3)
            self.assertGreaterEqual(result['applied'], 1)
            self.assertEqual(result['control_phase_at_stop'], 0)

    def test_main_ee_shares_body_owner_typed_touch_and_cancel(self):
        import importlib
        import flatbuffers
        from . import run
        from locomotion_aorta.end_effector_types import DeviceModel, PositionDomain
        def module(name): return importlib.import_module('lowlevel.ee.'+name)
        def write(path, obj):
            builder = flatbuffers.Builder(1024); builder.Finish(obj.Pack(builder))
            path.write_bytes(builder.Output())
        build = Path(os.environ['MOCK_BUILD_DIR']).resolve()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary); spec = {}
            for side, number, n in (('left',1,6), ('right',2,1)):
                region = module('TouchRegionInfo').TouchRegionInfoT(regionId=1, name='synthetic',
                    frame='synthetic', sensorType=1, shape=[1], components=['raw'], dtype=1,
                    unit='1', payloadOffset=0, payloadBytes=2, encodingVersion=1)
                info = module('EndEffectorInfo').EndEffectorInfoT(protocolVersion=1,
                    descriptorRef=module('DescriptorRef').DescriptorRefT(100), layoutId=7, side=number,
                    deviceModel=DeviceModel.BRAINCO_REVO2 if number==1 else DeviceModel.AGX,
                    axisNames=['synthetic'+str(i) for i in range(n)],
                    positionDomain=[PositionDomain.JOINT_ANGLE if number==1 else PositionDomain.TOTAL_OPENING]*n,
                    torqueDomain=[0]*n, supportedModes=[2]*n, supportedCommandFields=[65]*n,
                    supportedStateFields=[1]*n, validLimits=[1]*n, posMin=[0.]*n, posMax=[.0625]*n,
                    isMock=[True]*n, touchAvailable=number==1,
                    touchRegions=[region] if number==1 else [])
                write(root/(side+'-info.bin'), info)
                spec[side] = dict(info=side+'-info.bin', initial_pos=[.02]*n,
                    state_hz=100, info_hz=10, observation_sources=[[2,number]])
                if number == 1:
                    touch = module('EndEffectorTouch').EndEffectorTouchT(protocolVersion=1,
                        descriptorRef=module('DescriptorRef').DescriptorRefT(100), layoutId=7, side=1,
                        touchStateId=1, regionStamps=[module('RegionStamp').RegionStampT(validity=1)],
                        payload=[128,255])
                    write(root/'left-touch.bin', touch)
                    spec[side].update(touch='left-touch.bin', touch_hz=20)
            fixture = root/'fixture.json'; fixture.write_text(json.dumps(spec))
            output = root/'run'
            args = run.parser().parse_args(['--mode','production-topics','--build-dir',str(build),
                '--output',str(output),'--seconds','15','--ee-fixture',str(fixture),'--allow-client',
                '--client-command',sys.executable,str(Path(__file__).resolve()),'--ee-client'])
            rc = run.run(args)
            self.assertEqual(rc,0,'\n'.join(p.read_text() for p in output.glob('*.log')))
            result = json.loads((output/'mock-result.json').read_text())
            self.assertEqual(result['endpoint_rc'],0)
            self.assertEqual(result['parse_errors'],0)
            self.assertEqual(result['ee_accepted'],2)
            self.assertEqual(result['ee_rejected'],0)
            self.assertEqual(result['touch_requests'],2)
            self.assertEqual(result['control_phase_at_stop'],0)
            self.assertFalse((output/'auxiliary.json').exists())

    def test_real_typed_waits_keep_body_live_then_cancel_join_and_close(self):
        from . import run
        for mode in ('late_reply', 'no_reply', 'no_provider'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary); output = root/'run'; evidence = root/'touch-wait.json'
                args = run.parser().parse_args(['--mode','production-topics','--build-dir',
                    str(Path(os.environ['MOCK_BUILD_DIR']).resolve()), '--output',str(output),
                    '--seconds','15','--allow-client','--client-command',sys.executable,
                    str(Path(__file__).with_name('test_touch_wait_ipc.py').resolve()),'--client',mode,str(evidence)])
                rc = run.run(args)
                logs = '\n'.join(p.read_text() for p in output.glob('*.log'))
                # Keep partial source-specific evidence even when this case fails.
                if evidence.is_file():
                    print('TOUCH_WAIT_IPC '+evidence.read_text(), flush=True)
                self.assertEqual(rc,0,logs)
                report = json.loads(evidence.read_text())
                self.assertEqual(report['outcome'],'ASSERTIONS_COMPLETED')
                result = json.loads((output/'mock-result.json').read_text())
                self.assertEqual(result['endpoint_rc'],0)
                self.assertEqual(result['parse_errors'],0)
                self.assertGreaterEqual(result['accepted'],3)
                self.assertEqual(result['control_phase_at_stop'],0)


if __name__ == '__main__':
    if sys.argv[1:2] == ['--client']:
        client()
    elif sys.argv[1:2] == ['--ee-client']:
        ee_client()
    else:
        unittest.main()
