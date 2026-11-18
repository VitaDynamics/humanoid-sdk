"""Real SDK native codec/ORT hooks; synthetic payloads, no device commands."""
from dataclasses import asdict, fields, replace
from pathlib import Path
import tempfile
import os
import threading
import uuid
import unittest
from unittest.mock import patch

from locomotion_aorta import _body_cmd_native as native
from locomotion_aorta._native_body_codec import NativeBodyPublisher
from locomotion_aorta.state_arrays import decode_body_state_arrays
from locomotion_aorta.trace import Trace, Kind, Outcome
from locomotion_aorta.trace.reader import Reader
from test_native_body_cmd import Publisher
from test_body_codec import make_command, make_state
from test_end_effector_codec import state_object, touch_object, pack
import test_sdk_trace as sdk_cases


class NativeTraceHooks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'native.ftrace'

    def read(self):
        reader = Reader(self.path)
        rows = list(reader)
        self.assertTrue(reader.integrity.complete, reader.integrity.issues)
        stages = {name:i+1 for i,name in enumerate(reader.integrity.run_header['stages'])}
        return rows, stages

    def test_body_encoder_reuses_original_values_and_records_float32_separately(self):
        import body_cmd_schema_meta
        pub = Publisher()
        publisher = NativeBodyPublisher(pub, body_cmd_schema_meta)
        self.addCleanup(publisher.close)
        command = make_command()
        command = replace(command, joints=(replace(command.joints[0],q=.1),)+command.joints[1:])
        def publish(value):
            publisher.publish(value, guard=lambda send:send())
            return value
        with Trace(self.path) as trace:
            # Accessing joints after native encode would fail. The returned
            # command exposes scalar key fields but denies a duplicate snapshot.
            class Returned:
                def __getattr__(self, name):
                    if name == 'joints':
                        raise AssertionError('Trace reread joints after encode')
                    return getattr(command, name)
            def send(value):
                publish(value)
                return Returned()
            trace.call('body_publish',send,command)
        rows,stages=self.read()
        encoded=next(r for r in rows if r['stage']==stages['body_encode'])
        returned=next(r for r in rows if r['kind']==Kind.BODY_COMMAND and r['outcome']==Outcome.OK)
        self.assertEqual(returned['values'][0],.1)
        self.assertNotEqual(encoded['values'][0],.1)
        self.assertEqual(encoded['command_sequence'],command.sequence)
        self.assertEqual(returned['command_sequence'],command.sequence)
        self.assertEqual(encoded['attempt_id'],returned['attempt_id'])
        self.assertEqual(len(pub.sent),1)

    def test_encode_success_is_not_submit_success(self):
        import body_cmd_schema_meta
        pub=Publisher(); publisher=NativeBodyPublisher(pub,body_cmd_schema_meta)
        self.addCleanup(publisher.close)
        primary=RuntimeError('final owner gate refused')
        def fail(_):raise primary
        def send(command):
            publisher.publish(command,guard=fail)
            return command
        with Trace(self.path) as trace:
            with self.assertRaises(RuntimeError) as caught:
                trace.call('body_publish',send,make_command())
            self.assertIs(caught.exception,primary)
        rows,stages=self.read()
        self.assertEqual(sum(r['stage']==stages['body_encode'] for r in rows),1)
        self.assertFalse(any(r['kind']==Kind.BODY_COMMAND and r['outcome']==Outcome.OK for r in rows))
        self.assertEqual(pub.sent,[])

    def test_returned_scalar_reentry_keeps_outer_snapshot_and_marks_nested_capture_loss(self):
        import body_cmd_schema_meta
        publisher = NativeBodyPublisher(Publisher(), body_cmd_schema_meta)
        self.addCleanup(publisher.close)
        outer = make_command()
        inner = replace(outer, sequence=outer.sequence+1,
                        joints=(replace(outer.joints[0], q=.25),)+outer.joints[1:])
        with Trace(self.path) as trace:
            def send(command):
                publisher.publish(command, guard=lambda submit: submit())
                return command
            class Returned:
                @property
                def runtime_id(self):
                    trace.call('body_publish', send, inner)
                    return outer.runtime_id

                def __getattr__(self, name):
                    return getattr(outer, name)

            def outer_send(command):
                send(command)
                return Returned()
            trace.call('body_publish', outer_send, outer)
        reader = Reader(self.path)
        rows = list(reader)
        self.assertFalse(reader.integrity.complete)
        self.assertIn('incomplete_producer', reader.integrity.issues)
        commands = {r['command_sequence']: r for r in rows
                    if r['kind'] == Kind.BODY_COMMAND and r['outcome'] == Outcome.OK}
        self.assertEqual(set(commands), {outer.sequence})
        self.assertEqual(commands[outer.sequence]['values'][0], outer.joints[0].q)
        self.assertEqual(len(publisher.publisher.sent), 2)

    def test_decoded_body_and_cached_read_link_exact_snapshot_without_joint_getters(self):
        payload,_=make_state()
        with Trace(self.path) as trace:
            state=trace.decode(decode_body_state_arrays,payload)
            self.assertNotIn('_trace_ref',{f.name for f in fields(state)})
            self.assertNotIn('_trace_ref',asdict(state))
            dto=state.body_state()
            self.assertEqual(dto._trace_ref,state._trace_ref)
            self.assertFalse(hasattr(replace(dto), '_trace_ref'))
            class Selected:
                def __getattr__(self,name):
                    if name in ('joints','q','dq','tau_est','joint_stamps_us'):
                        raise AssertionError('duplicate decode snapshot')
                    return getattr(state,name)
            selected=Selected()
            trace.call('body_read',lambda:selected)
            trace.call('body_read',lambda:selected)
        rows,stages=self.read()
        decoded=next(r for r in rows if r['stage']==stages['body_decode'] and r['kind']==Kind.BODY_FEEDBACK)
        reads=[r for r in rows if r['kind']==Kind.BODY_READ and r['outcome']==Outcome.OK]
        self.assertEqual(len(reads),2)
        self.assertEqual([r['reference_id'] for r in reads],[decoded['attempt_id']]*2)
        self.assertTrue(all(r['state_id']==state.state_id and not r['values'] for r in reads))
        self.assertEqual(len(decoded['values']),124)

    def test_real_ort_run_is_separate_and_linked_to_outer_model_span(self):
        import numpy as np
        from demo_onnx.tests.test_session import write_model
        from locomotion_aorta._onnx import Session
        from locomotion_aorta.trace.summary import summarize_spans
        model=Path(self.temp.name)/'model.onnx';write_model(model)
        session=Session(str(model),intra_threads=1,inter_threads=1)
        with Trace(self.path,stages=('actor',)) as trace:
            with trace.span('actor',iteration_id=2) as span:
                result=session.run('obs','action',np.array([[0.,2.,4.]],np.float32))
        np.testing.assert_array_equal(result,[[.5,3.,2.]])
        rows,stages=self.read()
        inner=next(r for r in rows if r['stage']==stages['native_inference'])
        outer=next(r for r in rows if r['kind']==Kind.USER_SPAN)
        self.assertEqual(inner['inference_id'],span.inference_id)
        self.assertEqual(inner['span_id'],outer['span_id'])
        self.assertGreaterEqual(inner['timestamp_us'],outer['timestamp_us'])
        self.assertLessEqual(inner['timestamp_us']+inner['duration_us'],outer['timestamp_us']+outer['duration_us'])
        groups={r['stage']:r for r in summarize_spans(self.path)['stages']}
        self.assertEqual(groups['actor/native_run']['observed_spans'],1)
        self.assertEqual(groups['actor/native_run']['p50_us'],inner['duration_us'])

    def test_ee_encoder_reuses_target_and_keeps_selected_observation(self):
        import end_effector_cmd_schema_meta
        from locomotion_aorta._native_body_codec import NativeEEPublisher
        from locomotion_aorta.end_effector_types import EndEffectorSubmission, CommandKey, SampleRef
        from test_end_effector_codec import decode, info_object, target
        from locomotion_aorta._end_effector_codec import prepare_target
        from unittest.mock import Mock
        info=decode('info',info_object())
        pub=Publisher()
        pub.close=Mock()
        sender=NativeEEPublisher(pub,end_effector_cmd_schema_meta,info,((2,1),))
        self.addCleanup(sender.close)
        command=EndEffectorSubmission(100,7,1,13,CommandKey(8,9),
            prepare_target(info,replace(target(),pos=(.03,))),
            (SampleRef(100,2,1,44,123,15),))
        class Returned:
            def __getattr__(self,name):
                if name=='target':raise AssertionError('target reread after native encode')
                return getattr(command,name)
        class Endpoint:
            side='left'
            def publish(self,value):
                sender.publish_bytes(sender.encode(value))
                return Returned()
        with Trace(self.path) as trace:
            trace.call('endpoint_publish',Endpoint().publish,command)
        rows,stages=self.read()
        encoded=next(r for r in rows if r['stage']==stages['endpoint_encode'])
        sent=next(r for r in rows if r['kind']==Kind.EFFECTOR_COMMAND and r['outcome']==Outcome.OK)
        self.assertEqual(sent['values'][0],.03)
        self.assertNotEqual(encoded['values'][0],.03)
        self.assertEqual(sent['command_sequence'],9)
        link=next(r for r in rows if r['kind']==Kind.TRACE_LINK)
        self.assertEqual((link['state_id'],link['command_sequence']),(44,9))

    def test_real_ort_failure_keeps_error_and_failed_inner_span(self):
        import numpy as np
        from demo_onnx.tests.test_session import write_model
        from locomotion_aorta._onnx import Session
        model = Path(self.temp.name) / 'missing-input.onnx'
        write_model(model, two_inputs=True)
        session = Session(str(model), intra_threads=1, inter_threads=1)
        with Trace(self.path, stages=('actor',)) as trace:
            with self.assertRaisesRegex(RuntimeError, 'Missing Input: extra'):
                with trace.span('actor'):
                    session.run('obs', 'action', np.zeros((1, 3), np.float32))
        rows, stages = self.read()
        inner = next(r for r in rows if r['stage'] == stages['native_inference'])
        outer = next(r for r in rows if r['kind'] == Kind.USER_SPAN)
        self.assertEqual((inner['outcome'], outer['outcome']),
                         (Outcome.FAILED, Outcome.FAILED))
        self.assertEqual(inner['span_id'], outer['span_id'])

    def test_touch_receipt_references_native_payload_without_rereading_it(self):
        from locomotion_aorta import _end_effector_codec as codec
        payload=pack(touch_object())
        with Trace(self.path) as trace:
            value=trace.decode(codec.decode_touch,payload,_stage='touch_decode')
            class Received:
                def __getattr__(self,name):
                    if name in ('payload','region_stamps'):raise AssertionError('Touch reread')
                    return getattr(value,name)
            trace.received('touch',Received())
            self.assertNotIn('_trace_ref',asdict(value))
        rows,stages=self.read()
        data=next(r for r in rows if r['stage']==stages['touch_decode'] and r['kind']==Kind.INPUT_EVENT)
        received=next(r for r in rows if r['stage']==stages['touch_received'])
        self.assertEqual(received['reference_id'],data['attempt_id'])
        self.assertEqual(data['integers'][3],len(value.payload))
        raw=b''.join(i.to_bytes(8,'little') for i in data['integers'][4:])
        self.assertEqual(raw[:len(value.payload)],value.payload)

    def test_reference_from_another_recording_is_not_reused(self):
        from locomotion_aorta import _end_effector_codec as codec
        with Trace(Path(self.temp.name)/'old.ftrace') as old:
            state=old.decode(codec.decode_state,pack(state_object()),_stage='endpoint_decode')
        with Trace(self.path) as trace:
            trace.received('state',state)
        rows,_=self.read()
        data=next(r for r in rows if r['kind']==Kind.EFFECTOR_FEEDBACK)
        self.assertEqual(data['reference_id'],0)
        self.assertEqual(len(data['values']),6)
        self.assertEqual(data['values'][0],state.pos[0])

    def test_foreign_runtime_ee_decode_is_not_admitted(self):
        case=sdk_cases.SdkTraceTests();case.setUp()
        try:
            h,node=case.harness();ee=case.endpoint(h,node)
            accepted=ee.state.state_id
            node.emit('left','state',state_object(runtime=101,sample=77))
            self.assertEqual(ee.state.state_id,accepted)
            h.controller.close()
            reader=Reader(case.path);rows=list(reader)
            self.assertTrue(reader.integrity.complete,reader.integrity.issues)
            stage=reader.integrity.run_header['stages'].index('endpoint_decode')+1
            self.assertTrue(any(r['stage']==stage and r['runtime_id']==101 and r['state_id']==77 for r in rows))
            self.assertFalse(any(r['reference_id'] and r['runtime_id']==101 and r['outcome']==Outcome.OK for r in rows))
            self.assertTrue(any(r['reference_id'] and r['runtime_id']==101 and r['outcome']==Outcome.REJECTED for r in rows))
        finally:
            case.doCleanups()

    def test_real_aorta_callbacks_reuse_native_thread_channel_across_python_tls_reset(self):
        from aorta import Node
        from locomotion_aorta import _end_effector_codec as codec
        done = threading.Event()
        observed, errors = [], []
        group = uuid.uuid4().hex
        with patch.dict(os.environ, {'AORTA_GROUP': group}):
            with Trace(self.path, stages=('main',)) as trace:
                with trace.span('main'):
                    pass
                with Node('trace-callback-regression', group=group) as node:
                    def receive(payload, _context):
                        try:
                            sample = trace.decode(codec.decode_state, payload,
                                                  _stage='endpoint_decode')
                            trace.received('state', sample)
                            observed.append((threading.get_native_id(), sample.state_id))
                        except BaseException as error:
                            errors.append(error)
                        finally:
                            done.set()
                    subscriber = node.create_subscriber('/sdk/test/trace-callback', receive)
                    publisher = node.create_publisher('/sdk/test/trace-callback')
                    for sample_id in range(1, 25):
                        done.clear()
                        publisher.publish_bytes(pack(state_object(sample=sample_id)))
                        self.assertTrue(done.wait(2), 'actual native callback did not arrive')
                    subscriber.close()
                    publisher.close()
                self.assertEqual(errors, [])
                self.assertEqual([item[1] for item in observed], list(range(1, 25)))
                native_threads = {item[0] for item in observed}
                self.assertLess(len(native_threads), len(observed))
                self.assertEqual(len(trace._threads), 1 + len(native_threads))
        rows, stages = self.read()
        decoded = [r for r in rows if r['kind'] == Kind.EFFECTOR_FEEDBACK
                   and r['stage'] == stages['endpoint_decode']]
        accepted = [r for r in rows if r['kind'] == Kind.EFFECTOR_FEEDBACK
                    and r['stage'] == stages['endpoint_feedback']]
        # The writer drains independent producer rings; global file order is
        # not callback order. Preserve exact completeness and reference checks
        # without imposing a nonexistent cross-producer serialization promise.
        self.assertCountEqual([r['state_id'] for r in decoded], range(1, 25))
        self.assertCountEqual([r['state_id'] for r in accepted], range(1, 25))
        by_sample = {r['state_id']: r for r in decoded}
        for row in accepted:
            original = by_sample[row['state_id']]
            self.assertEqual(row['reference_id'], original['attempt_id'])
            self.assertEqual(row['producer'], original['producer'])
            self.assertEqual(row['runtime_id'], original['runtime_id'])
        for producer in {r['producer'] for r in decoded}:
            samples = [r['state_id'] for r in decoded if r['producer'] == producer]
            self.assertEqual(samples, sorted(samples))
        self.assertEqual(len({r['producer'] for r in decoded}), len(native_threads))

    def test_native_thread_token_lifetime_does_not_reuse_a_retired_threads_channel(self):
        from locomotion_aorta.trace import _trace_native
        original = _trace_native.thread_token()
        self.assertEqual(_trace_native.thread_token(), original)
        tokens = []
        for _ in range(12):
            thread = threading.Thread(target=lambda: tokens.append(_trace_native.thread_token()))
            thread.start()
            thread.join(2)
            self.assertFalse(thread.is_alive())
        self.assertEqual(len(set(tokens)), 12)
        self.assertNotIn(original, tokens)


if __name__=='__main__':unittest.main()
