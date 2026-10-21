"""Owned NumPy body feedback. Importing this module does not import NumPy."""
from dataclasses import dataclass
from functools import cached_property
from typing import Any

from .body_types import BodyState, BodyJointState, BodyImuState, BodyImuSample


@dataclass(frozen=True, eq=False)
class BodyStateArrays:
    """Independent read-only arrays; every wire integer keeps its full width.

    joint_stamps_us columns: acquired, ingested, assembled.
    imu_stamps_us columns: sampled, acquired, ingested, assembled.
    imu_values columns: quaternion WXYZ, gyroscope XYZ, accelerometer XYZ,
    RPY in degrees. Rows with imu_state_present == 0 have no IMU state;
    their zero storage is not a measured value. Receipt clocks are local
    monotonic/realtime; the context stamp is the Core pull assembly clock.
    """
    state_id: int
    q: Any
    dq: Any
    tau_est: Any
    joint_stamps_us: Any
    error_code: Any
    imu_values: Any
    imu_stamps_us: Any
    imu_location: Any
    imu_status: Any
    imu_state_present: Any
    protocol_version: int
    roster_version: int
    ts_state_real_us: int
    ts_state_pub_us: int
    ts_consumed_us: int
    attitude_valid: bool
    applied_session_id: int
    applied_sequence: int
    ts_first_output_us: int
    received_monotonic_ns: int = 0
    received_realtime_ns: int = 0
    pull_context_receive_stamp_ns: int = 0

    @property
    def stamp_ns(self):
        return self.ts_state_real_us * 1000

    @cached_property
    def joints(self):
        """Explicit compatibility materialization; policies can use arrays directly."""
        return tuple(BodyJointState(*(int(v) for v in self.joint_stamps_us[i]),
                     float(self.q[i]), float(self.dq[i]), float(self.tau_est[i]),
                     int(self.error_code[i])) for i in range(len(self.q)))

    @cached_property
    def imu_states(self):
        result = []
        for i, values in enumerate(self.imu_values):
            state = (BodyImuState(*(tuple(float(v) for v in values[a:b])
                       for a, b in ((0, 4), (4, 7), (7, 10), (10, 13))))
                     if self.imu_state_present[i] else None)
            result.append(BodyImuSample(int(self.imu_location[i]), state,
                          int(self.imu_status[i]), *(int(v) for v in self.imu_stamps_us[i])))
        return tuple(result)

    def body_state(self):
        return BodyState(self.state_id, self.joints, self.protocol_version,
                         self.roster_version, self.ts_state_real_us,
                         self.ts_state_pub_us, self.ts_consumed_us,
                         self.attitude_valid, self.imu_states,
                         self.applied_session_id, self.applied_sequence,
                         self.ts_first_output_us)


def decode_body_state_arrays(payload, *, measure=False):
    """Decode pinned BODY2 bytes, preserving absent vectors and unknown values."""
    from ._native_body_codec import _state_decoder
    from . import _body_cmd_native
    _state_decoder()  # Shared BFBS identity guard; no silent Python fallback.
    values, timing = _body_cmd_native.decode_state_arrays(payload, 0, measure)
    result = BodyStateArrays(**values)
    return (result, timing) if measure else result
