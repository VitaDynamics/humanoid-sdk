"""Native/pinned-Python state goldens; no Aorta nodes or control requests."""
import dataclasses
import importlib
import math
import os
import random
import struct
import unittest
import time
import uuid
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest import mock

import flatbuffers
from locomotion_aorta import BodyStateSnapshot
from locomotion_aorta._body_codec import _decode_body_state_python
from locomotion_aorta._native_body_codec import decode_body_state_native, _state_decoder
decode_body_state = decode_body_state_native
from test_body_codec import make_state, module

try:
    from locomotion_aorta import _body_cmd_native as native
except ImportError:
    native = None


def state_buffer(seed, *, count=31, missing_joints=False, missing_imus=False,
                 missing_sample=False, protocol=3):
    rng = random.Random(seed)
    builder = flatbuffers.Builder(1)
    imu_module = module('BodyImuSample')
    imus = []
    floats = [0., -0., 2**-149, -2**-149, math.nan, math.inf, -math.inf]
    floats += [rng.uniform(-100, 100) for _ in range(16)]
    for i in range(seed % 5):
        imu_module.BodyImuSampleStart(builder)
        imu_module.BodyImuSampleAddLocation(builder, [0, 1, 2, 255][i % 4])
        imu_module.BodyImuSampleAddStatus(builder, [0, 1, 7, 255][i % 4])
        for name in ('TsSampledUs', 'TsAcquiredHwUs', 'TsIngestedUs', 'TsAssembledUs'):
            getattr(imu_module, 'BodyImuSampleAdd' + name)(builder, rng.randrange(2**64))
        if not missing_sample:
            groups = [tuple(rng.choice(floats) for _ in range(n)) for n in (4, 3, 3, 3)]
            imu_module.BodyImuSampleAddState(builder, module('BodyImuState').CreateBodyImuState(builder, *groups))
        imus.append(imu_module.BodyImuSampleEnd(builder))
    m = module('BodyState')
    m.BodyStateStartImuStatesVector(builder, len(imus))
    for v in reversed(imus): builder.PrependUOffsetTRelative(v)
    imu_vector = builder.EndVector()
    m.BodyStateStartJointsVector(builder, count)
    for _ in range(count):
        module('BodyJointState').CreateBodyJointState(builder,
            *(rng.randrange(2**64) for _ in range(3)),
            *(rng.choice(floats) for _ in range(3)), rng.choice([0, 8, 255, 0x80000000, 0xffffffff]))
    joints = builder.EndVector()
    m.BodyStateStart(builder)
    m.BodyStateAddProtocolVersion(builder, protocol)
    m.BodyStateAddRosterVersion(builder, rng.randrange(2**32))
    for name in ('StateId', 'TsStateRealUs', 'TsStatePubUs', 'TsConsumedUs',
                 'AppliedSessionId', 'AppliedSequence', 'TsFirstOutputUs', 'TsControlUs'):
        getattr(m, 'BodyStateAdd' + name)(builder, rng.randrange(2**64))
    m.BodyStateAddRuntimeId(builder, 2**64-1)
    m.BodyStateAddAppliedKeyValid(builder, bool(seed % 2))
    m.BodyStateAddAssociationSourceType(builder, seed % 256)
    m.BodyStateAddAttitudeValid(builder, bool(seed % 2))
    if not missing_joints: m.BodyStateAddJoints(builder, joints)
    if not missing_imus: m.BodyStateAddImuStates(builder, imu_vector)
    builder.Finish(m.BodyStateEnd(builder))
    return bytes(builder.Output())


class _NativeStateAssertions:
    def same(self, expected, actual):
        self.assertIs(type(expected), type(actual))
        if dataclasses.is_dataclass(expected):
            for f in dataclasses.fields(expected):
                with self.subTest(field=f.name): self.same(getattr(expected, f.name), getattr(actual, f.name))
        elif isinstance(expected, tuple):
            self.assertEqual(len(expected), len(actual))
            for a, b in zip(expected, actual): self.same(a, b)
        elif isinstance(expected, float):
            if math.isnan(expected): self.assertTrue(math.isnan(actual))
            else: self.assertEqual(struct.pack('<d', expected), struct.pack('<d', actual))
        else: self.assertEqual(expected, actual)

@unittest.skipIf(native is None, 'installed native state codec required; pkg requires zero skips')
class NativeBodyStateTest(_NativeStateAssertions, unittest.TestCase):
    def compare(self, data, *, offset=0):
        view = module('BodyState').BodyState.GetRootAs(data, offset)
        expected = _decode_body_state_python(view)
        actual = decode_body_state_native(view)
        self.same(expected, actual)
        return actual

    def test_original_generated_golden_all_fields_and_imu_order(self):
        data, _ = make_state()
        self.compare(data)

    def test_seeded_random_golden_256_full_uint_unknown_nan_and_clocks(self):
        for seed in range(256):
            with self.subTest(seed=seed): self.compare(state_buffer(seed))

    def test_absent_joints_imus_and_optional_sample_keep_defaults(self):
        for kwargs in ({'missing_imus':True}, {'missing_sample':True}):
            with self.subTest(**kwargs): self.compare(state_buffer(3, **kwargs))
        missing = state_buffer(3, missing_joints=True)
        with self.assertRaisesRegex(ValueError, '31 joint slots'):
            _decode_body_state_python(module('BodyState').BodyState.GetRootAs(missing))
        with self.assertRaisesRegex(ValueError, '31 joint slots'):
            decode_body_state_native(missing)
        b = flatbuffers.Builder(1);m = module('BodyState')
        m.BodyStateStart(b);m.BodyStateAddProtocolVersion(b, 3);m.BodyStateAddRuntimeId(b, 11);b.Finish(m.BodyStateEnd(b))
        with self.assertRaisesRegex(ValueError, '31 joint slots'):
            self.compare(bytes(b.Output()))

    def test_wrong_protocol_and_present_wrong_joint_lengths_reject(self):
        for kwargs in ({'protocol':0}, {'protocol':1}, {'protocol':2}, {'protocol':4}, {'protocol':0xffffffff},
                       {'count':0}, {'count':30}, {'count':32}, {'count':42}):
            view = module('BodyState').BodyState.GetRootAs(state_buffer(1, **kwargs))
            with self.subTest(**kwargs):
                with self.assertRaises(ValueError): _decode_body_state_python(view)
                with self.assertRaises(ValueError): decode_body_state(view)

    def test_valid_zero_key_all_source_association_reaches_native_dto(self):
        m = module('BodyState')
        for source in (0, 1, 2, 3, 4, 5, 6, 255):
            with self.subTest(source=source):
                obj = m.BodyStateT.InitFromObj(make_state()[1])
                obj.runtimeId = 2**64-1
                obj.tsControlUs, obj.appliedKeyValid, obj.associationSourceType = 2**64-2, True, source
                b = flatbuffers.Builder(1); b.Finish(obj.Pack(b))
                state = self.compare(bytes(b.Output()))
                self.assertEqual((state.applied_session_id, state.applied_sequence, state.ts_first_output_us), (0, 0, 0))
                self.assertTrue(state.applied_key_valid)
                self.assertEqual(state.association_source_type, source)
                self.assertEqual(state.runtime_id, 2**64-1)
                self.assertEqual(state.ts_control_us, 2**64-2)

    def test_present_zero_runtime_is_rejected_by_python_and_native(self):
        m = module('BodyState')
        obj = m.BodyStateT.InitFromObj(make_state()[1])
        obj.runtimeId = 0
        builder = flatbuffers.Builder(1)
        builder.Finish(obj.Pack(builder))
        payload = bytes(builder.Output())
        with self.assertRaisesRegex(ValueError, 'nonzero runtime_id'):
            _decode_body_state_python(m.BodyState.GetRootAs(payload))
        with self.assertRaisesRegex(ValueError, 'nonzero runtime_id'):
            native.decode_state(payload, 0)

    def test_truncated_corrupt_and_invalid_table_offsets_reject(self):
        data = state_buffer(3);pos = struct.unpack_from('<I', data)[0]
        for n in range(len(data)):
            with self.subTest(length=n), self.assertRaises(ValueError):
                native.decode_state(data[:n], pos)
        for off in (1, 2, 3, len(data), len(data)+1, 2**63):
            with self.subTest(offset=off), self.assertRaises(ValueError): native.decode_state(data, off)
        corrupt = bytearray(data);struct.pack_into('<i', corrupt, pos, 0x7fffffff)
        with self.assertRaises(ValueError): native.decode_state(corrupt, pos)

    def test_bytes_bytearray_memoryview_embedded_offset_and_owned_results(self):
        data = state_buffer(3);expected = self.compare(data)
        for value in (data, bytearray(data), memoryview(data), memoryview(bytearray(data))):
            self.same(expected, self.compare(value))
        self.same(expected, self.compare(bytes(8)+data, offset=8))
        source = bytearray(data);result = decode_body_state(module('BodyState').BodyState.GetRootAs(source))
        source[:] = bytes(len(source));self.same(expected, result)
        source = bytearray(data)
        borrowed = memoryview(source)
        result = decode_body_state(module('BodyState').BodyState.GetRootAs(borrowed))
        source[:] = bytes(len(source))
        borrowed.release()
        del source, borrowed
        self.same(expected, result)

    def test_parallel_decodes_have_owned_independent_state_with_gil_held(self):
        payloads = [state_buffer(i) for i in range(32)]
        with ThreadPoolExecutor(4) as pool:
            states = list(pool.map(self.compare, payloads))
        self.assertEqual(len({id(s) for s in states}), 32)

    def test_production_pull_dto_uses_native_and_matches_python_field_by_field(self):
        from test_pull import PullTest, S
        from locomotion_aorta import PullController
        from locomotion_aorta.transport import BODY_STATE_TOPIC
        from body_fixtures import robot_info_reply
        t = PullTest().transport(
            node=S(create_subscriber_typed=lambda *a, **kw: S(close=lambda: None)),
            get_robot_info=lambda _: dataclasses.replace(robot_info_reply(), runtime_id=2**64-1))
        controller = PullController(client_id=7, _transport=t)
        self.addCleanup(controller.close)
        controller.get_robot_info(.1)
        _state_decoder.cache_clear()
        self.addCleanup(_state_decoder.cache_clear)
        with mock.patch.object(native, 'decode_state', wraps=native.decode_state) as decode:
            for seed in range(32):
                data = state_buffer(seed)
                t._pulls[BODY_STATE_TOPIC].samples.append(
                    S(payload=data, context=S(receive_stamp_ns=42)))
                with mock.patch('locomotion_aorta.pull.decode_body_state_arrays',
                                side_effect=AssertionError('array materialization used')):
                    actual = controller.wait_body_state(.1)
                expected = _decode_body_state_python(module('BodyState').BodyState.GetRootAs(data))
                self.same(expected, actual)
                self.assertIs(controller.wait_body_state(0), actual)
                self.assertEqual(decode.call_count, seed + 1)
                decode.assert_called_with(data, 0)
        _state_decoder.cache_clear()
        with mock.patch('body_state_schema_meta.SCHEMA_HASH_HEX', 'wrong'):
            with self.assertRaisesRegex(RuntimeError, 'pinned BodyState3'):
                decode_body_state_native(data)
        _state_decoder.cache_clear()

    def test_bad_pull_frame_raises_once_and_preserves_previous_snapshot(self):
        from test_pull import PullTest, S
        from locomotion_aorta.transport import BODY_STATE_TOPIC
        for arrays in (True, False):
            with self.subTest(arrays=arrays):
                t = PullTest().transport()
                read = t.wait_state if arrays else t._wait_body_state
                ring = t._pulls[BODY_STATE_TOPIC]
                def put(payload):
                    ring.samples.append(S(payload=payload, context=S(receive_stamp_ns=42)))
                put(state_buffer(3))
                first = read(0)
                receipt = t._latest_sample
                for bad in (b'bad', state_buffer(3, missing_joints=True),
                            state_buffer(3, count=30)):
                    put(bad)
                    with self.assertRaises(ValueError):
                        read(0)
                    self.assertIs(t._latest_sample, receipt)
                    self.assertIs(read(0), first)
                put(state_buffer(4))
                self.assertNotEqual(read(0).state_id, first.state_id)
                empty = PullTest().transport()
                empty._pulls[BODY_STATE_TOPIC].samples.append(
                    S(payload=b'bad', context=S(receive_stamp_ns=42)))
                empty_read = empty.wait_state if arrays else empty._wait_body_state
                with self.assertRaises(ValueError): empty_read(0)
                with self.assertRaises(TimeoutError): empty_read(0)

    def test_stale_native_protocol_cannot_pass_even_with_matching_old_metadata(self):
        _state_decoder.cache_clear()
        self.addCleanup(_state_decoder.cache_clear)
        with mock.patch.object(native, 'BODY_STATE_PROTOCOL_VERSION', 2):
            with self.assertRaisesRegex(RuntimeError, 'pinned BodyState3'):
                decode_body_state_native(state_buffer(3))
        _state_decoder.cache_clear()

    def test_native_entry_returns_only_the_state_dto(self):
        data = state_buffer(3);view = module('BodyState').BodyState.GetRootAs(data)
        state = native.decode_state(data, view._tab.Pos)
        self.same(_decode_body_state_python(view), state)


@unittest.skipIf(native is None, 'installed native state codec required; pkg requires zero skips')
class NativeBodyPullTest(_NativeStateAssertions, unittest.TestCase):
    """Actual Core InlinePull -> native borrow -> original SDK codec/gate.

    Only discovery uses the existing service-reply fixture. No raw sample,
    slot/POD, Core callback or decoder is replaced by a fake backend.
    """

    def source(self, frames):
        from dataclasses import replace
        from body_fixtures import robot_info_reply
        from locomotion_aorta import PullStateSource
        from locomotion_aorta.transport import BODY_STATE_TOPIC
        group = uuid.uuid4().hex
        self.assertEqual(len(group.encode('utf-8')), 32)
        group_env = mock.patch.dict(os.environ, {'AORTA_GROUP': group})
        group_env.start()
        self.addCleanup(group_env.stop)
        source = PullStateSource(group=group,
            native_body_rx={'slot_count': 2,
                'max_payload_bytes': max(map(len, frames)),
                'max_slices': 3, 'poll_interval_s': .001})
        self.addCleanup(source.close)
        self.assertEqual(source.transport.node.group(), group)
        reply = replace(robot_info_reply(), runtime_id=11)
        with mock.patch.object(source.transport, 'get_robot_info', return_value=reply):
            self.assertIs(source.get_robot_info(.1), reply)
        pub = source.transport.node.create_publisher(BODY_STATE_TOPIC)
        return source, pub, source.transport._native_body_rx

    def receive(self, reader):
        # Bounded real local IPC arrival wait, not a timing assertion.
        deadline, pause = time.monotonic()+2, threading.Event()
        while True:
            sample = reader.try_recv()
            if sample is not None:
                return sample
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                self.fail('actual Core sample did not arrive')
            pause.wait(min(remaining, .001))

    def test_actual_source_keeps_raw_for_lazy_dto_arrays_and_owned_results(self):
        data, _ = make_state(runtime_id=11)
        source, pub, reader = self.source([data])
        pub.publish_bytes(data)
        arrays = source.wait_state(2)
        transport = source.transport
        raw = transport._latest_sample[0]
        self.assertIsInstance(raw, native.NativeBodyPullSample)
        dto = transport._wait_body_state(0)
        self.assertIs(transport._latest_sample[0], raw)
        self.assertEqual(arrays.state_id, dto.state_id)
        receipts = (arrays.received_monotonic_ns, arrays.received_realtime_ns,
                    arrays.pull_context_receive_stamp_ns)
        self.assertEqual(receipts[2], raw.receive_stamp_ns)
        self.assertIs(source.wait_state(0), arrays)
        self.assertEqual(receipts, (arrays.received_monotonic_ns,
            arrays.received_realtime_ns, arrays.pull_context_receive_stamp_ns))
        raw = None
        source.close()
        self.assertEqual(float(arrays.q[0]), dto.joints[0].q)
        self.same(arrays.body_state(), dto)

    def test_busy_preserves_original_queue_and_foreign_candidate_preserves_cache(self):
        from aorta.errors import BusyError
        first, _ = make_state(runtime_id=11, error_codes=(1,)*31)
        foreign, _ = make_state(runtime_id=12, error_codes=(2,)*31)
        last, _ = make_state(runtime_id=11, error_codes=(3,)*31)
        source, pub, reader = self.source([first, foreign, last])
        pub.publish_bytes(first)
        a = source.wait_state(2)
        accepted = source.transport._latest_sample
        pub.publish_bytes(foreign)
        candidate = self.receive(reader)
        self.assertIs(source.transport._accept_state(
            (candidate, time.time_ns(), time.monotonic_ns()), arrays=True), a)
        self.assertIs(source.transport._latest_sample, accepted)
        pub.publish_bytes(last)
        with self.assertRaises(BusyError):
            reader.try_recv()  # Both real slots held: no dequeue.
        candidate = None
        b = self.receive(reader)
        got = source.transport._accept_state(
            (b, time.time_ns(), time.monotonic_ns()), arrays=True)
        self.assertEqual(int(got.error_code[0]), 3)
        self.assertEqual(int(a.error_code[0]), 1)
        b = accepted = None

    def test_close_busy_keeps_real_borrow_readable_then_explicit_drain(self):
        from aorta.errors import BusyError, NotFoundError
        data, _ = make_state(runtime_id=11)
        source, pub, reader = self.source([data])
        pub.publish_bytes(data)
        arrays = source.wait_state(2)
        raw = source.transport._latest_sample[0]
        with self.assertRaises(BusyError):
            source.close()
        self.assertFalse(source.transport._closed)
        self.assertEqual(decode_body_state_native(raw).state_id, arrays.state_id)
        with self.assertRaises(NotFoundError):
            reader.try_recv()  # Permanently closed, never a new sample.
        raw = None
        source.close()
        self.assertTrue(source.transport._closed)
        self.assertEqual(len(arrays.q), 31)  # Independent owning output survived.

    def test_real_over_budget_does_not_replace_old_or_expose_stale_out(self):
        from aorta.errors import InvalidArgumentError
        data, _ = make_state(runtime_id=11)
        source, pub, reader = self.source([data])
        pub.publish_bytes(data)
        old = source.wait_state(2)
        accepted = source.transport._latest_sample
        pub.publish_bytes(data+b'\x00')  # Whole-wire B is exactly len(data).
        with self.assertRaises(InvalidArgumentError):
            self.receive(reader)
        self.assertIs(source.transport._latest_sample, accepted)
        self.assertIs(source.wait_state(0), old)
        pub.publish_bytes(data)
        fresh = self.receive(reader)
        self.assertIsInstance(fresh, native.NativeBodyPullSample)
        self.assertEqual(decode_body_state_native(fresh).runtime_id, 11)
        fresh = accepted = None

    def test_real_late_borrow_is_released_before_timeout_not_committed(self):
        data, _ = make_state(runtime_id=11)
        source, pub, reader = self.source([data])
        offset = [0.0]
        real_clock, start = time.monotonic, time.monotonic()
        original = native.NativeBodyPullReader.try_recv

        def late(actual_reader):
            sample = original(actual_reader)
            if sample is not None:
                offset[0] = 20.0  # After the REAL Core borrow, before SDK return.
            return sample

        pub.publish_bytes(data)
        with mock.patch('locomotion_aorta.pull.time.monotonic',
                        side_effect=lambda: real_clock()-start+offset[0]), \
             mock.patch.object(native.NativeBodyPullReader, 'try_recv', late):
            with self.assertRaises(TimeoutError):
                source.wait_state(2)
        self.assertEqual(offset[0], 20.0)  # Not a no-data timeout false positive.
        self.assertIsNone(source.transport._latest_sample)
        pub.publish_bytes(data)
        first = self.receive(reader)
        pub.publish_bytes(data)
        second = self.receive(reader)  # Both slots available: late token returned.
        try:
            self.same(decode_body_state_native(first), decode_body_state_native(second))
        finally:
            first = second = None

    def test_partial_cold_ctor_frees_real_first_slot_and_preserves_receiver(self):
        import ctypes
        from locomotion_aorta.transport import BODY_STATE_TOPIC
        data, _ = make_state(runtime_id=11)
        source, pub, _ = self.source([data])
        binding = source.transport._pulls[BODY_STATE_TOPIC].native_rx_binding(
            slot_count=2, max_payload_bytes=len(data), max_slices=3)
        self.addCleanup(binding.close)
        signature = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p)
        real_free = signature(binding.free_address)
        frees = []

        @signature
        def observe_free(handle):
            # Test-local observer delegates to the REAL same-Core free; no
            # fake token/POD, release result or backend is supplied.
            rc = real_free(handle)
            frees.append((handle, rc))
            return rc

        class FailSecond:
            calls = 0
            free_address = ctypes.cast(observe_free, ctypes.c_void_p).value

            def __getattr__(self, name):
                return getattr(binding, name)

            def acquire_slot(self):
                self.calls += 1
                if self.calls == 2:
                    raise RuntimeError('second cold acquisition stopped')
                return binding.acquire_slot()

        failed = FailSecond()
        with self.assertRaisesRegex(RuntimeError, 'second cold acquisition stopped'):
            native.NativeBodyPullReader(failed)
        self.assertEqual(failed.calls, 2)
        self.assertEqual(len(frees), 1)
        self.assertTrue(frees[0][0])
        self.assertEqual(frees[0][1], 0)
        pub.publish_bytes(data)
        self.assertEqual(source.wait_state(2).runtime_id, 11)

    def test_saved_decode_identity_tracebacks_release_only_unaccepted_borrow(self):
        data, _ = make_state(runtime_id=11, error_codes=(3,)*31)
        zero_runtime, _ = make_state(runtime_id=0)
        foreign, _ = make_state(runtime_id=12)
        # Both actual codec entry frames and the no-old-cache identity failure.
        for arrays, bad, seed_old in ((True, zero_runtime, True),
                                     (False, data[:3], True),
                                     (True, foreign, False)):
            with self.subTest(arrays=arrays, seed_old=seed_old):
                source, pub, reader = self.source([data, zero_runtime, foreign])
                transport = source.transport
                if seed_old:
                    pub.publish_bytes(data)
                    old = source.wait_state(2)
                    accepted = transport._latest_sample
                else:
                    old = accepted = None
                pub.publish_bytes(bad)
                candidate = self.receive(reader)
                errors = []
                try:
                    transport._accept_state(
                        (candidate, time.time_ns(), time.monotonic_ns()), arrays=arrays)
                except ValueError as error:
                    errors.append(error)  # Keep the ORIGINAL traceback intact.
                self.assertEqual(len(errors), 1)
                self.assertIn('BodyState', str(errors[0]))
                self.assertIsNotNone(errors[0].__traceback__)
                self.assertIs(transport._latest_sample, accepted)
                if accepted is not None:
                    self.assertEqual(decode_body_state_native(accepted[0]).state_id, old.state_id)
                stale = candidate
                candidate = None
                with self.assertRaisesRegex(ValueError, 'released'):
                    decode_body_state_native(stale)
                # With old raw held, M=2 has exactly ONE reusable slot. Without
                # old raw, two accepted frames expose a retained foreign token.
                for _ in range(1 if seed_old else 2):
                    pub.publish_bytes(data)
                    fresh = self.receive(reader)
                    got = transport._accept_state(
                        (fresh, time.time_ns(), time.monotonic_ns()), arrays=True)
                    self.assertEqual(int(got.error_code[0]), 3)
                    # Old wrapper must not observe or release this reused slot.
                    self.assertEqual(stale._discard(), 0)
                    with self.assertRaisesRegex(ValueError, 'released'):
                        _ = stale.receive_stamp_ns
                    self.assertEqual(decode_body_state_native(fresh).runtime_id, 11)
                    fresh = None
                accepted = None  # Drop only our own legitimate old selection.
                source.close()  # Must succeed while errors/stale remain retained.
                self.assertIsNotNone(errors[0].__traceback__)

    def test_saved_final_closed_traceback_does_not_hold_provisional_core_slot(self):
        from aorta.errors import BusyError
        from locomotion_aorta import ControllerClosedError
        data, _ = make_state(runtime_id=11)
        source, pub, _ = self.source([data])
        entered, errors = [False], []
        real_clock = time.time_ns

        def close_at_receipt():
            if not entered[0]:
                entered[0] = True
                with self.assertRaises(BusyError):
                    source.close()  # Actual new borrow exists, not yet accepted.
            return real_clock()

        pub.publish_bytes(data)
        with mock.patch('locomotion_aorta.pull.time.time_ns', side_effect=close_at_receipt):
            try:
                source.wait_state(2)
            except ControllerClosedError as error:
                errors.append(error)
        self.assertTrue(entered[0])
        self.assertEqual(len(errors), 1)
        self.assertIsNotNone(errors[0].__traceback__)
        self.assertIsNone(source.transport._latest_sample)
        source.close()  # No GC or traceback surgery before actual drain.
        self.assertTrue(source.transport._closed)
        self.assertIsNotNone(errors[0].__traceback__)

    def test_cache_cleared_by_close_does_not_reclassify_true_selected_raw(self):
        from aorta.errors import BusyError, ResultCode
        from locomotion_aorta import ControllerClosedError
        data, _ = make_state(runtime_id=11)
        source, pub, reader = self.source([data])
        pub.publish_bytes(data)
        owned = source.wait_state(2)
        selected = source.transport._latest_sample
        with self.assertRaises(BusyError):
            source.close()
        self.assertIsNone(source.transport._latest_sample)
        errors = []
        try:
            source.transport._accept_state(selected, arrays=False)
        except ControllerClosedError as error:
            errors.append(error)
        self.assertEqual(len(errors), 1)
        self.assertIsNotNone(errors[0].__traceback__)
        self.assertEqual(selected[0]._discard(), 0)  # Accepted selection is NOT ended.
        self.assertEqual(decode_body_state_native(selected[0]).state_id, owned.state_id)
        self.assertEqual(reader.close(), ResultCode.BUSY)  # Our true selection exists.
        # Only this true accepted-selection case drops its own external refs;
        # the rejected-candidate tests above keep original errors through close.
        selected = errors = None
        source.close()
        self.assertTrue(source.transport._closed)
