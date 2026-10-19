"""Selected body31 columns over an immutable, frame-owned public payload."""
from __future__ import annotations

from array import array
from collections.abc import Mapping, Iterator
from dataclasses import dataclass
from types import MappingProxyType

from ._body_codec import (
    _vector_field_data, _BODY_JOINTS_SLOT, _JOINT_STATE, _APPLIED_JOINTS_SLOT,
    _dto, _getter, unpack_body_cmd,
)
from .body_types import BodyAppliedCmd, BodyImuSample, BodyImuState
from .state import BodyStateMetadata, FrameToken


@dataclass(frozen=True, slots=True)
class _Buffers(Mapping):
    _data: Mapping

    def __getitem__(self, key) -> memoryview:
        fmt, data = self._data[key]
        # Each access owns a view wrapper: a reader's release() cannot invalidate others.
        return memoryview(data).cast(fmt)

    def __iter__(self) -> Iterator[str]:
        return iter(self._data)

    def __len__(self) -> int:
        return len(self._data)


@dataclass(frozen=True, slots=True)
class JointColumns:
    token: FrameToken
    received_monotonic_ns: int
    received_realtime_ns: int
    metadata: BodyStateMetadata
    joint_indices: tuple[int, ...]
    values: Mapping[str, memoryview]
    _columns: _Buffers

    @property
    def body_index(self): return self._columns['body_index']
    @property
    def populated(self): return self._columns['populated']
    @property
    def is_mock(self): return self._columns['is_mock']
    @property
    def status(self): return self._columns['status']
    @property
    def valid_fields(self): return self._columns['valid_fields']
    @property
    def ts_acquired_hw_us(self): return self._columns['ts_acquired_hw_us']
    @property
    def ts_ingested_us(self): return self._columns['ts_ingested_us']
    @property
    def ts_assembled_us(self): return self._columns['ts_assembled_us']
    @property
    def lost(self): return self._columns['lost']


@dataclass(frozen=True, slots=True)
class JointSelection:
    _snapshot: object
    _joint_indices: tuple[int, ...]
    _fields: tuple[str, ...]
    _imu_location: int | None

    def columns(self) -> JointColumns:
        snapshot = self._snapshot
        return snapshot._cached(('columns', self._joint_indices, self._fields),
                                lambda: _columns(snapshot, self._joint_indices, self._fields))

    def imu_sample(self) -> BodyImuSample | None:
        if self._imu_location is None:
            return None
        snapshot = self._snapshot
        return snapshot._cached(('imu', self._imu_location),
                                lambda: _imu_sample(snapshot._view(), self._imu_location))


def select(snapshot, *, joint_indices, fields, imu_location):
    if (not isinstance(joint_indices, tuple)
            or any(isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < 31
                   for i in joint_indices)
            or len(set(joint_indices)) != len(joint_indices)):
        raise ValueError('joint_indices must be a tuple of distinct body31 indices')
    if (not isinstance(fields, tuple) or not fields
            or any(not isinstance(f, str) or f not in ('q', 'dq') for f in fields)
            or len(set(fields)) != len(fields)):
        raise ValueError('fields must be a non-empty tuple of distinct q/dq names')
    if imu_location is not None and (isinstance(imu_location, bool)
                                    or not isinstance(imu_location, int)):
        raise ValueError('imu_location must be an integer or None')
    return snapshot._cached(('select', joint_indices, fields, imu_location),
                            lambda: JointSelection(snapshot, joint_indices, fields, imu_location))


_LAYOUT = {
    'body_index': ('H', 0), 'populated': ('B', 1), 'is_mock': ('B', 2),
    'status': ('B', 3), 'valid_fields': ('H', 4), 'ts_acquired_hw_us': ('Q', 5),
    'ts_ingested_us': ('Q', 6), 'ts_assembled_us': ('Q', 7), 'lost': ('I', 13),
    'q': ('f', 8), 'dq': ('f', 9),
}


def _columns(snapshot, indices, fields):
    view = snapshot._view()
    data = _vector_field_data(view, _BODY_JOINTS_SLOT, _JOINT_STATE.size, 8)
    rows = () if data is None else tuple(_JOINT_STATE.unpack_from(data, i * _JOINT_STATE.size)
                                        for i in indices)
    buffers = {}
    for name in tuple(n for n in _LAYOUT if n not in ('q', 'dq')) + fields:
        fmt, position = _LAYOUT[name]
        column = array(fmt, (row[position] for row in rows))
        if column.itemsize != {'B': 1, 'H': 2, 'I': 4, 'Q': 8, 'f': 4}[fmt]:
            raise RuntimeError('unsupported stdlib array element width')
        buffers[name] = (fmt, column.tobytes())
    return JointColumns(snapshot.token, snapshot.received_monotonic_ns,
                        snapshot.received_realtime_ns, snapshot.metadata, indices,
                        _Buffers(MappingProxyType({f: buffers[f] for f in fields})),
                        _Buffers(MappingProxyType({n: b for n, b in buffers.items() if n not in fields})))


def _imu_sample(view, location):
    selected = None
    for i in range(view.ImuStatesLength()):
        sample = view.ImuStates(i)
        if sample.Location() == location:
            if selected is not None:
                raise ValueError('ambiguous IMU location')
            selected = sample
    if selected is None:
        return None
    state = selected.State()
    imu = None if state is None else BodyImuState(**{
        name: tuple(getattr(state, _getter(name))(j)
                    for j in range(getattr(state, _getter(name) + 'Length')()))
        for name in ('quaternion', 'gyroscope', 'accelerometer', 'rpy')})
    return _dto(selected, BodyImuSample, state=imu)


def applied_cmd(snapshot) -> BodyAppliedCmd | None:
    def decode():
        applied = snapshot._view().AppliedCmd()
        return None if applied is None else _dto(
            applied, BodyAppliedCmd,
            joints=unpack_body_cmd(applied, slot=_APPLIED_JOINTS_SLOT))
    return snapshot._cached('applied_cmd', decode)
