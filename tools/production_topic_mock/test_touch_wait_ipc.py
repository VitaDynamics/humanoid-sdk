"""Real C-ABI touch waits beside compiled Mock Body traffic, on private IPC only.

The test peer supplies synthetic EE Info/State and a deferred typed service.
It does not grant ownership: the original compiled Mock owns Body/EXTERNAL.
No production fault flag, fake Node, fake RPC result or alternate SDK lock.
"""
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch


def touch_wait_client(mode, output):
    import aorta
    import flatbuffers
    from aorta.sys.AortaHeader import AortaHeaderT
    from locomotion_aorta import EndEffectorClient, PullController
    from locomotion_aorta import _end_effector_codec as codec
    from locomotion_aorta.end_effector_types import (ControlMode, DeviceModel, Field,
        PositionDomain, SampleStatus, SampleStream, ServiceStatus,
        TouchPayloadType, TouchSetReason)

    if mode not in ('late_reply', 'no_reply', 'no_provider'):
        raise ValueError('unknown IPC scenario')
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('actual_ee_cli', root/'examples/end_effector/end_effector_external.py')
    demo = importlib.util.module_from_spec(spec); spec.loader.exec_module(demo)
    trace, held, futures = [], [], []
    lock = threading.Lock()
    def mark(event, **values):
        with lock:
            row = dict(event=event, time=time.monotonic(), **values)
            trace.append(row)
        print('TOUCH_WAIT_EVENT '+json.dumps(row,sort_keys=True), flush=True)
    def rows(event):
        with lock:
            return [dict(row) for row in trace if row['event'] == event]
    def module(name):
        return importlib.import_module('lowlevel.ee.'+name)
    def pack(obj, sequence):
        obj.aortaHeader = AortaHeaderT()
        obj.aortaHeader.sequence = sequence
        obj.aortaHeader.publishStampNs = time.time_ns()
        obj.aortaHeader.publisherNode = 'synthetic_touch_wait_fixture'
        builder = flatbuffers.Builder(1024); builder.Finish(obj.Pack(builder))
        return bytes(builder.Output())

    class ObservedPool(ThreadPoolExecutor):
        # Delegate to the real executor; instrumentation never fabricates a result.
        def submit(self, function, *args, **kwargs):
            def invoke():
                mark('worker_start')
                try:
                    result = function(*args, **kwargs)
                    mark('worker_return')
                    return result
                except BaseException as error:
                    mark('worker_error', type=type(error).__name__, error=str(error),
                         code=getattr(error, 'code', None))
                    raise
            future = super().submit(invoke); futures.append(future)
            return future
        def shutdown(self, *args, **kwargs):
            mark('join_begin')
            try:
                return super().shutdown(*args, **kwargs)
            finally:
                mark('join_end')

    controller = PullController(group=os.environ['AORTA_GROUP'])
    endpoint = service = None
    publishers = []
    outcome = 'NOT_COMPLETED'
    try:
        actual_body = controller.wait_body_state(5)
        ref = actual_body.descriptor_ref
        assert actual_body.protocol_version == 3
        bindings, rpc_bindings = codec.load_bindings(), codec.load_touch_bindings()
        wire_ref = module('DescriptorRef').DescriptorRefT(ref.robot_runtime_id)
        info = module('EndEffectorInfo').EndEffectorInfoT(protocolVersion=1, descriptorRef=wire_ref,
            layoutId=7, side=1, deviceModel=DeviceModel.AGX, firmware='synthetic', serial='synthetic',
            axisNames=['synthetic'], positionDomain=[PositionDomain.TOTAL_OPENING], torqueDomain=[0], supportedModes=[1 << ControlMode.POSITION],
            supportedCommandFields=[Field.POS | Field.MODE], supportedStateFields=[Field.POS], validLimits=[Field.POS],
            posMin=[0.], posMax=[.0625], isMock=[True], touchAvailable=True,
            touchRegions=[module('TouchRegionInfo').TouchRegionInfoT(regionId=1, name='synthetic',
                frame='synthetic', sensorType=1, shape=[1], components=['raw'], dtype=TouchPayloadType.UINT16_LE,
                unit='1', payloadOffset=0, payloadBytes=2, encodingVersion=1)])
        node = controller.transport.node
        endpoint = EndEffectorClient('left', connection=controller,
                                     observation_sources=((SampleStream.EE_STATE, 1),))
        topic = '/sdk/foot_humanoid/end_effector/left'
        info_pub = node.create_publisher_typed(bindings.metas['info'], topic+'/info', qos=aorta.QoS.state_update())
        state_pub = node.create_publisher_typed(bindings.metas['state'], topic+'/state', qos=aorta.QoS.sensor_data())
        publishers.extend((info_pub, state_pub))
        assert info_pub.wait_for_matching(3000) and state_pub.wait_for_matching(3000)
        info_pub.publish_bytes(pack(info, 1))
        endpoint.wait_info(3)
        sample_id = 0
        def publish_state():
            nonlocal sample_id
            sample_id += 1
            state = module('EndEffectorState').EndEffectorStateT(protocolVersion=1,
                descriptorRef=wire_ref, layoutId=7, side=1, stateId=sample_id, sampleStatus=SampleStatus.OK,
                validFields=[Field.POS], pos=[.03125], ready=True, outputAllowed=True,
                inhibitReason=0, touchEnabled=False, statusValidMask=15)
            # Fixture sample identity only; no invented robot acquisition time.
            state_pub.publish_bytes(pack(state, sample_id))
        publish_state(); endpoint.wait_state(3)

        def retain(request, responder):
            with lock:
                held.append((request, responder))
            mark('request_received')
        if mode != 'no_provider':
            service = node.create_service_typed_view(
                rpc_bindings.request.EndEffectorSetTouchRequest.GetRootAs, topic+'/set_touch', retain,
                request_schema_meta=rpc_bindings.request_meta)

        # Observe calls around the actual Python binding/C-ABI, without replacing it.
        create_rpc = node.create_client_typed
        def observed_rpc(*args, **kwargs):
            rpc = create_rpc(*args, **kwargs)
            if args[0] != topic+'/set_touch':
                return rpc
            raw, call, free = rpc.raw(), rpc.call, rpc.close
            matching = raw.wait_for_matching
            def wait_matching(timeout):
                mark('matching_begin')
                result = matching(timeout)
                mark('matching_end', matched=result)
                return result
            def invoke(*args, **kwargs):
                mark('call_begin')
                try:
                    result = call(*args, **kwargs)
                    mark('call_return')
                    return result
                except BaseException as error:
                    mark('call_error', type=type(error).__name__, error=str(error), code=getattr(error, 'code', None))
                    raise
            def close_rpc():
                mark('rpc_free_begin')
                try: return free()
                finally: mark('rpc_free_end')
            raw.wait_for_matching, rpc.call, rpc.close = wait_matching, invoke, close_rpc
            return rpc
        body_publish, cancel = controller.publish, controller.exit_external
        ee_publish = endpoint.publish
        def body_send(command):
            sent = body_publish(command)
            mark('body_send', sample=command.input_state_id, sequence=sent.sequence)
            return sent
        def ee_send(*args, **kwargs):
            mark('ee_send')
            return ee_publish(*args, **kwargs)
        def cancel_then_reply(timeout):
            mark('cancel_begin')
            result = cancel(timeout)
            mark('cancel_end', action=result.current_action)
            if mode == 'late_reply':
                assert len(held) == 1
                request, responder = held[0]
                response = rpc_bindings.response.EndEffectorSetTouchResponseT(
                    protocolVersion=1, descriptorRef=wire_ref, side=1, status=ServiceStatus.SUCCESS,
                    reason=TouchSetReason.NONE)
                mark('late_reply_begin')
                responder.reply_bytes(pack(response, 1))
                mark('late_reply_end')
            return result
        interrupted = False
        def paced_sleep(seconds):
            nonlocal interrupted
            time.sleep(seconds)
            publish_state()
            began = rows('matching_begin' if mode == 'no_provider' else 'call_begin')
            if began and not interrupted:
                body = [r for r in rows('body_send') if r['time'] > began[0]['time']]
                if len(body) >= 3 and (mode == 'no_provider' or rows('request_received')):
                    assert futures and not futures[0].done(), 'RPC completed before cancellation exercise'
                    interrupted = True
                    mark('interrupt', body_frames=len(body))
                    raise KeyboardInterrupt('IPC cancellation while actual native wait is pending')
        with patch.object(node, 'create_client_typed', side_effect=observed_rpc), \
             patch.object(controller, 'publish', side_effect=body_send), \
             patch.object(controller, 'exit_external', side_effect=cancel_then_reply), \
             patch.object(endpoint, 'publish', side_effect=ee_send), \
             patch.object(demo, 'ThreadPoolExecutor', ObservedPool):
            try:
                demo.run(controller, [endpoint], {'left':demo.position_target(endpoint.info, (.03125,), unit='m')},
                         timeout=1., duration=.1, touch='on', sleep=paced_sleep)
            except KeyboardInterrupt:
                assert interrupted
            else:
                raise AssertionError('did not cancel while actual native wait was pending')
        assert len(futures) == 1 and futures[0].done() and futures[0].exception() is not None
        assert not rows('ee_send') and not rows('worker_return')
        assert len(rows('cancel_begin')) == 1 and rows('cancel_end')[0]['action'] == 'PASSIVE'
        native_begin = rows('matching_begin' if mode == 'no_provider' else 'call_begin')[0]['time']
        native_end = (rows('matching_end') if mode == 'no_provider' else
                      rows('call_return') + rows('call_error'))[0]['time']
        body = [r for r in rows('body_send') if native_begin < r['time'] < native_end]
        assert len(body) >= 3 and len({r['sample'] for r in body}) >= 2
        assert rows('cancel_end')[0]['time'] < native_end <= rows('rpc_free_begin')[0]['time']
        assert rows('rpc_free_end')[0]['time'] <= rows('join_end')[0]['time']
        assert all(r['time'] < rows('cancel_begin')[0]['time'] for r in rows('body_send'))
        if mode == 'no_provider':
            assert rows('matching_end')[0]['matched'] is False and not rows('call_begin') and not held
        elif mode == 'no_reply':
            assert len(held) == 1 and rows('call_error')[0]['code'] == -3  # Actual C-ABI TIMEOUT.
        else:
            assert len(rows('call_return')) == 1 and not rows('call_error')
            assert rows('late_reply_begin')[0]['time'] > rows('cancel_end')[0]['time']
        assert controller.status.current_action == 'PASSIVE'
        outcome = 'PRE_CLOSE_ASSERTIONS_COMPLETED'
    finally:
        # The per-call native client is freed by set_touch after the call ends;
        # EE/connection lifetime ends only after the real executor has joined.
        for _request, responder in held:
            if responder.is_valid(): responder.raw().release()
        if service is not None: service.close()
        for publisher in publishers: publisher.close()
        if endpoint is not None:
            mark('endpoint_close_begin'); endpoint.close(); mark('endpoint_close_end')
        mark('connection_close_begin'); controller.close(); mark('connection_close_end')
        try:
            if outcome == 'PRE_CLOSE_ASSERTIONS_COMPLETED':
                assert rows('join_end')[0]['time'] < rows('endpoint_close_begin')[0]['time']
                assert rows('endpoint_close_end')[0]['time'] < rows('connection_close_begin')[0]['time']
                outcome = 'ASSERTIONS_COMPLETED'
        finally:
            Path(output).write_text(json.dumps(dict(synthetic=True, scenario=mode,
                outcome=outcome, trace=trace, limits=['no device/HIL', 'no declared hard timeout/free upper bound']), indent=2)+'\n')


if __name__ == '__main__':
    if sys.argv[1:2] == ['--client']:
        touch_wait_client(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit("use the existing production_topic_mock.test_ipc suite")
