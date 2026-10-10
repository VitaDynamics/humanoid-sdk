"""Equivalence to the original codec using the actual pinned message wheels."""

import math
import random
import struct
import unittest
from unittest.mock import patch

import flatbuffers
import aorta.sys.AortaHeader as header_module
import lowlevel.HumanoidLowState as state_module
import lowlevel.HumanoidMotorState as motor_state_module
import lowlevel.MotorCmd as motor_cmd_module

from locomotion_aorta import HumanoidLowState, LowCmd, MotorCommand, MotorState
from locomotion_aorta import transport


def reference_fill_low_cmd(bindings, command, builder, header):
    # Original generated-struct path from SDK main 839020f9879d.
    module = bindings.low_cmd_module
    module.LowCmdStartMotorCmdVector(builder, len(command.motor_cmds))
    for motor in reversed(command.motor_cmds):
        motor_cmd_module.CreateMotorCmd(
            builder, motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd
        )
    vector = builder.EndVector()
    module.LowCmdStart(builder)
    module.LowCmdAddAortaHeader(builder, header)
    module.LowCmdAddMotorCmd(builder, vector)
    module.LowCmdAddCmdId(builder, command.cmd_id)
    module.LowCmdAddExternalVersion(builder, command.external_version)
    module.LowCmdAddExternalSessionId(builder, command.external_session_id)
    module.LowCmdAddExternalSequence(builder, command.external_sequence)
    return module.LowCmdEnd(builder)


def encode(bindings, command, fill, initial_size=256, include_header=True):
    builder = flatbuffers.Builder(initial_size)
    header = 0
    if include_header:
        name = builder.CreateString("codec-test")
        header_module.AortaHeaderStart(builder)
        header_module.AortaHeaderAddPublisherNode(builder, name)
        header_module.AortaHeaderAddPublishStampNs(builder, 1_789_988_776_655_443_000)
        header_module.AortaHeaderAddSequence(builder, 123456789)
        header = header_module.AortaHeaderEnd(builder)
    builder.Finish(fill(bindings, command, builder, header))
    return builder.Output()


def build_state(motors, *, reverse_fields=False, force_defaults=False,
                include_vector=True, extra_field=False):
    builder = flatbuffers.Builder(1)
    builder.ForceDefaults(force_defaults)
    offsets = []
    for fields in motors:
        # One appended field simulates a compatible future table producer.
        builder.StartObject(22 if extra_field else 21)
        if extra_field:
            builder.PrependUint32Slot(21, 123, 0)
        items = list(fields.items())
        if reverse_fields:
            items.reverse()
        for name, value in items:
            getattr(motor_state_module, "HumanoidMotorStateAdd" + name)(builder, value)
        offsets.append(motor_state_module.HumanoidMotorStateEnd(builder))
    if include_vector:
        state_module.HumanoidLowStateStartMotorStatesVector(builder, len(offsets))
        for offset in reversed(offsets):
            builder.PrependUOffsetTRelative(offset)
        vector = builder.EndVector()
    state_module.HumanoidLowStateStart(builder)
    if include_vector:
        state_module.HumanoidLowStateAddMotorStates(builder, vector)
    state_module.HumanoidLowStateAddStateId(builder, 65000)
    state_module.HumanoidLowStateAddTsStateReal(builder, 1_789_988_776_655_443)
    state_module.HumanoidLowStateAddAttitudeValid(builder, False)
    builder.Finish(state_module.HumanoidLowStateEnd(builder))
    return builder.Output()


def reference_decode(view):
    motors = []
    for index in range(view.MotorStatesLength()):
        motor = view.MotorStates(index)
        motors.append(MotorState() if motor is None else MotorState(
            q=float(motor.Q()), dq=float(motor.Dq()),
            tau_est=float(motor.TauEst()), temperature=float(motor.Temperature()),
        ))
    return HumanoidLowState(
        state_id=int(view.StateId()), stamp_ns=int(view.TsStateReal()) * 1000,
        motor_states=tuple(motors), attitude_valid=bool(view.AttitudeValid()),
    )


class MotorCommandCodecTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bindings = transport._load_aorta_bindings()

    def assert_same_wire(self, motors, initial_size=256, include_header=True):
        command = LowCmd(65535, tuple(motors), 1, 2**64 - 1, 2**64 - 2)
        expected = encode(self.bindings, command, reference_fill_low_cmd,
                          initial_size, include_header)
        actual = encode(self.bindings, command, transport._fill_low_cmd,
                        initial_size, include_header)
        self.assertEqual(actual, expected)
        view = self.bindings.low_cmd_module.LowCmd.GetRootAs(actual)
        self.assertEqual(view.MotorCmdLength(), len(motors))
        self.assertEqual(view.CmdId(), 65535)
        self.assertEqual(view.ExternalVersion(), 1)
        self.assertEqual(view.ExternalSessionId(), 2**64 - 1)
        self.assertEqual(view.ExternalSequence(), 2**64 - 2)
        if include_header:
            self.assertEqual(view.AortaHeader().PublisherNode(), b"codec-test")
            self.assertEqual(view.AortaHeader().Sequence(), 123456789)
            self.assertEqual(view.AortaHeader().PublishStampNs(),
                             1_789_988_776_655_443_000)
        return actual

    def test_empty_single_and_robot_lengths_preserve_wire_and_order(self):
        for count in (0, 1, 30, 40, 42, 44, 45):
            for size in (0, 1, 256, 4096):
                with self.subTest(count=count, size=size):
                    motors = [MotorCommand(i, i + .125, -i - .25,
                                           i * 2 + .5, i + 30., i + 1.5)
                              for i in range(count)]
                    self.assert_same_wire(motors, size)

    def test_defaults_without_header_and_ieee_float_values(self):
        self.assert_same_wire([MotorCommand()] * 42, include_header=False)
        self.assert_same_wire([
            MotorCommand(0, -0., math.inf, -math.inf, math.nan, 3.4e38),
            MotorCommand(255, 1e-44, -3.4e38, 0., -0., .25),
        ])

    def test_randomized_vectors_match_generated_encoder(self):
        randomizer = random.Random(42010)
        for _ in range(20):
            motors = [MotorCommand(randomizer.randrange(256),
                                   *(randomizer.uniform(-100, 100) for _ in range(5)))
                      for _ in range(randomizer.randrange(46))]
            self.assert_same_wire(motors)

    def test_invalid_modes_and_float_values_are_still_rejected(self):
        for motor in (MotorCommand(-1), MotorCommand(256), MotorCommand(1.5),
                      MotorCommand(q="invalid"), MotorCommand(q=1e100)):
            with self.subTest(motor=motor):
                command = LowCmd(1, (motor,))
                for fill in (reference_fill_low_cmd, transport._fill_low_cmd):
                    with self.assertRaises((TypeError, struct.error, OverflowError)):
                        encode(self.bindings, command, fill)

    def test_generated_per_motor_encoder_is_not_called(self):
        motors = [MotorCommand(10, q=.25)] * 42
        expected = self.assert_same_wire(motors)  # Positive equivalence control.
        with patch.object(motor_cmd_module, "CreateMotorCmd",
                          side_effect=AssertionError("per-slot encoding")):
            self.assertEqual(encode(self.bindings, LowCmd(65535, tuple(motors), 1,
                                                        2**64 - 1, 2**64 - 2),
                                    transport._fill_low_cmd), expected)


class MotorStateCodecTest(unittest.TestCase):
    def assert_same_state(self, data, offset=0):
        view = state_module.HumanoidLowState.GetRootAs(data, offset)
        expected, actual = reference_decode(view), transport._decode_humanoid_lowstate(view)
        self.assertIs(type(actual), type(expected))
        self.assertEqual((actual.state_id, actual.stamp_ns, actual.attitude_valid),
                         (expected.state_id, expected.stamp_ns, expected.attitude_valid))
        self.assertIsInstance(actual.motor_states, tuple)
        self.assertEqual(len(actual.motor_states), len(expected.motor_states))
        for a, b in zip(actual.motor_states, expected.motor_states):
            self.assertIs(type(a), type(b))
            for field in ("q", "dq", "tau_est", "temperature"):
                x, y = getattr(a, field), getattr(b, field)
                self.assertIsInstance(x, float)
                if math.isnan(y):
                    self.assertTrue(math.isnan(x), field)
                else:
                    self.assertEqual(struct.pack("<d", x), struct.pack("<d", y), field)
        return actual

    def test_44_distinct_slots_and_metadata(self):
        state = self.assert_same_state(build_state([
            dict(Q=i + .125, Dq=-i - .25, TauEst=i * 2 + .5,
                 Temperature=i - 22, Mode=1, SlotIndex=i,
                 TsAcquiredHwUs=1234567890 + i, TsIngestedUs=1234567990 + i)
            for i in range(44)
        ]))
        self.assertEqual(state.motor_states[43].q, 43.125)
        self.assertEqual(state.motor_states[0].temperature, -22.)

    def test_absent_empty_and_variable_vectors(self):
        self.assert_same_state(build_state([], include_vector=False))
        for count in (0, 1, 30, 39, 40, 42, 44, 45):
            with self.subTest(count=count):
                self.assert_same_state(build_state([dict(Q=.25)] * count))

    def test_omitted_fields_and_empty_tables_use_generated_defaults(self):
        state = self.assert_same_state(build_state([
            {}, dict(Q=1), dict(Dq=2, Temperature=-128),
            dict(TauEst=3), dict(Temperature=127),
        ] * 8))
        self.assertEqual(state.motor_states[0], MotorState())
        self.assert_same_state(build_state([{}] * 44))

    def test_reordered_and_appended_fields_keep_projection(self):
        motors = [dict(Q=i+.5, Dq=i+.25, TauEst=-i-1., Temperature=31,
                       Ddq=9., DqRaw=7., Mode=1, TsIngestedUs=2**40)
                  for i in range(44)]
        normal = self.assert_same_state(build_state(motors))
        for reverse, extra in ((True, False), (False, True), (True, True)):
            self.assertEqual(normal, self.assert_same_state(build_state(
                motors, reverse_fields=reverse, extra_field=extra)))

    def test_ieee_floats_signed_temperature_and_explicit_defaults(self):
        self.assert_same_state(build_state([
            dict(Q=-0., Dq=math.inf, TauEst=-math.inf, Temperature=-128),
            dict(Q=math.nan, Dq=-0., TauEst=3.4e38, Temperature=127),
            dict(Q=0., Dq=0., TauEst=0., Temperature=0),
        ] * 14, force_defaults=True))

    def test_randomized_defaults_and_layouts(self):
        randomizer = random.Random(44010)
        for _ in range(20):
            motors = []
            for _ in range(44):
                fields = dict(Q=randomizer.uniform(-100, 100),
                              Dq=randomizer.uniform(-100, 100),
                              TauEst=randomizer.uniform(-100, 100),
                              Temperature=randomizer.randrange(-128, 128))
                if randomizer.random() < .5:
                    fields.pop(randomizer.choice(list(fields)))
                motors.append(fields)
            self.assert_same_state(build_state(motors, reverse_fields=True))

    def test_bytes_bytearray_memoryview_and_nonzero_root_offset(self):
        data = build_state([dict(Q=1, Dq=2, TauEst=3, Temperature=4)] * 44)
        for kind in (bytes, bytearray, memoryview):
            self.assert_same_state(kind(data))
        self.assert_same_state(b"prefix!!" + data, offset=8)

    def test_typed_and_multidimensional_memoryviews_use_byte_offsets(self):
        data = build_state([dict(Q=1, Dq=2, TauEst=3, Temperature=4)] * 44)
        self.assert_same_state(data)  # Positive control for the same wire bytes.
        for fmt in ("I", "Q"):
            with self.subTest(format=fmt):
                self.assert_same_state(memoryview(data).cast(fmt))
                self.assert_same_state(memoryview(b"prefix!!" + data).cast(fmt), offset=8)
        self.assert_same_state(memoryview(data).cast("B", shape=(len(data) // 8, 8)))

    def test_no_generated_per_motor_accessors(self):
        data = build_state([dict(Q=1, Dq=2, TauEst=3, Temperature=4)] * 44)
        expected = self.assert_same_state(data)  # Positive equivalence control.
        with patch.object(state_module.HumanoidLowState, "MotorStates",
                          side_effect=AssertionError("per-slot table")), \
             patch.object(motor_state_module.HumanoidMotorState, "Q",
                          side_effect=AssertionError("per-field getter")):
            self.assertEqual(transport._decode_humanoid_lowstate(
                state_module.HumanoidLowState.GetRootAs(data)), expected)

    def test_corrupt_offsets_layouts_and_truncation_fail_closed(self):
        data = build_state([dict(Q=1, Dq=2, TauEst=3, Temperature=4)] * 44)
        self.assert_same_state(data)
        for corruption in ("count", "table", "null", "vtable", "length",
                           "scalar", "overlap", "object", "truncate"):
            with self.subTest(corruption=corruption):
                bad = bytearray(data)
                view = state_module.HumanoidLowState.GetRootAs(bad)
                first = view._tab.Vector(view._tab.Offset(20))
                pos = first + struct.unpack_from("<I", bad, first)[0]
                vtable = pos - struct.unpack_from("<i", bad, pos)[0]
                if corruption == "count":
                    struct.pack_into("<I", bad, first - 4, 2**32 - 1)
                elif corruption == "table":
                    struct.pack_into("<I", bad, first, 2**32 - 1)
                elif corruption == "null":
                    struct.pack_into("<I", bad, first, 0)
                elif corruption == "vtable":
                    struct.pack_into("<i", bad, pos, 2**31 - 1)
                elif corruption == "length":
                    struct.pack_into("<H", bad, vtable, 5)
                elif corruption == "scalar":
                    struct.pack_into("<H", bad, vtable + 12, 2)
                elif corruption == "overlap":
                    struct.pack_into("<H", bad, vtable + 14,
                                     struct.unpack_from("<H", bad, vtable + 12)[0])
                elif corruption == "object":
                    struct.pack_into("<H", bad, vtable + 2, 65535)
                else:
                    del bad[-1:]
                with self.assertRaises(ValueError):
                    transport._decode_humanoid_lowstate(view)


if __name__ == "__main__":
    unittest.main()
