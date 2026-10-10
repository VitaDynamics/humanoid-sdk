"""Bulk codecs for the pinned Aorta motor layouts, without changing the wire.

MotorCmd is a 24-byte struct. HumanoidMotorState is a table, so its field
offsets must be read from each message's vtables, not assumed contiguous.
"""

import struct
from functools import lru_cache
from typing import Any

from .types import MotorCommand, MotorState


_U32 = struct.Struct("<I")
_S32 = struct.Struct("<i")
_VTABLE_HEADER = struct.Struct("<HH")
_STATE_VECTOR_SLOT = 20  # motor_states, not the legacy motor_state.
# Vtable slots/types from aorta-msgs 2026.9.10+humanoid.7100871:
# Q, Dq, TauEst (float32), Temperature (signed int8).
_STATE_FIELDS = ((12, "f", 4), (14, "f", 4), (18, "f", 4), (26, "b", 1))


@lru_cache(maxsize=16)
def _command_packer(count: int) -> struct.Struct:
    return struct.Struct("<" + "B3xfffff" * count)


def create_motor_cmd_vector(
    builder: Any, module: Any, motors: tuple[MotorCommand, ...]
) -> int:
    # Preserve the generated encoder's uint8 range check / TypeError before
    # struct.pack (which otherwise raises struct.error for out-of-range modes).
    from flatbuffers.number_types import Uint8Flags, enforce_number

    fields = []
    for motor in motors:
        enforce_number(motor.mode, Uint8Flags)
        fields.extend((motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd))
    payload = _command_packer(len(motors)).pack(*fields)
    module.LowCmdStartMotorCmdVector(builder, len(motors))
    # StartVector reserves/aligns the structs. Copy once, using the pinned
    # FlatBuffers Builder.CreateByteVector head/Bytes convention.
    end = builder.Head()
    builder.head = end - len(payload)
    builder.Bytes[builder.Head():end] = payload
    return builder.EndVector()


def _check_bounds(start: int, width: int, size: int) -> None:
    if start < 0 or width < 0 or start + width > size:
        raise ValueError("HumanoidLowState field outside buffer")


def _state_plan(
    data: memoryview, vtable: int, size: int
) -> tuple[struct.Struct, tuple[int, ...], int]:
    _check_bounds(vtable, _VTABLE_HEADER.size, size)
    length, object_size = _VTABLE_HEADER.unpack_from(data, vtable)
    if length < 4 or length % 2 or object_size < 4:
        raise ValueError("invalid HumanoidMotorState vtable")
    _check_bounds(vtable, length, size)
    entries = struct.unpack_from("<" + "H" * (length // 2), data, vtable)
    fields = []
    for index, (slot, fmt, width) in enumerate(_STATE_FIELDS):
        offset = entries[slot // 2] if slot < length else 0
        if offset:
            if offset < 4 or offset + width > object_size:
                raise ValueError("HumanoidMotorState scalar outside table")
            fields.append((offset, index, fmt, width))
    fields.sort()  # Producers may add fields in a different order.
    indices = [len(fields)] * len(_STATE_FIELDS)  # Appended default zero.
    cursor, format_parts = 0, ["<"]
    for ordinal, (offset, index, fmt, width) in enumerate(fields):
        if offset < cursor:
            raise ValueError("overlapping HumanoidMotorState scalar fields")
        if offset > cursor:
            format_parts.append(str(offset - cursor) + "x")
        format_parts.append(fmt)
        indices[index] = ordinal
        cursor = offset + width
    return struct.Struct("".join(format_parts)), tuple(indices), object_size


def unpack_motor_states(view: Any) -> tuple[MotorState, ...]:
    data = memoryview(view._tab.Bytes).cast("B")
    root = view._tab.Pos
    size = len(data)
    offset = view._tab.Offset(_STATE_VECTOR_SLOT)
    if not offset:
        return ()
    pointer = root + offset
    _check_bounds(pointer, 4, size)
    vector = pointer + _U32.unpack_from(data, pointer)[0]
    _check_bounds(vector, 4, size)
    count = _U32.unpack_from(data, vector)[0]
    first = vector + 4
    _check_bounds(first, count * 4, size)
    offsets = struct.iter_unpack("<I", data[first:first + count * 4])
    plans = {}  # Message-local: never reuse addresses from an earlier buffer.
    motors = []
    for index, (relative,) in enumerate(offsets):
        if relative < 4:
            raise ValueError("invalid HumanoidMotorState table offset")
        pos = first + index * 4 + relative
        _check_bounds(pos, 4, size)
        vtable = pos - _S32.unpack_from(data, pos)[0]
        plan = plans.get(vtable)
        if plan is None:
            plan = _state_plan(data, vtable, size)
            plans[vtable] = plan
        packer, indices, object_size = plan
        _check_bounds(pos, object_size, size)
        values = packer.unpack_from(data, pos) + (0.0,)
        motors.append(MotorState(
            q=values[indices[0]],
            dq=values[indices[1]],
            tau_est=values[indices[2]],
            temperature=float(values[indices[3]]),
        ))
    return tuple(motors)
