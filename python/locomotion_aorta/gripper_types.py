"""Public single-DOF AGX DTOs for the approved T027 schema.

Opening values are total opening in metres. Torque is the signed host software
motor-side command in N·m: not gripping force N or a device effort limit. The
device's interpretation remains governed by its vendor protocol/firmware.
"""
from dataclasses import dataclass
from lowlevel.GripperLimitSource import GripperLimitSource
from lowlevel.GripperType import GripperType
from lowlevel.InhibitReason import InhibitReason


@dataclass(frozen=True, slots=True)
class GripperInfo:
    ts_info_us: int
    gripper_type: int
    model: str
    firmware: str
    side: int
    opening_min_m: float
    opening_max_m: float
    max_opening_rate_m_s: float
    hold_torque_nm: float
    ts_state_pub_us: int = 0
    opening_min_source: int = GripperLimitSource.UNKNOWN
    opening_max_source: int = GripperLimitSource.UNKNOWN
    max_opening_rate_source: int = GripperLimitSource.UNKNOWN
    torque_limit_nm: float = 0.0
    torque_limit_source: int = GripperLimitSource.UNKNOWN
    hold_torque_source: int = GripperLimitSource.UNKNOWN


@dataclass(frozen=True, slots=True)
class GripperCmd:
    session_id: int
    sequence: int
    gripper_type: int
    side: int
    q: float
    dq: float = 0.0
    torque_cmd_nm: float | None = None
    protocol_version: int = 1
    cmd_id: int = 0
    input_state_id: int = 0
    input_state_ts_real_us: int = 0
    ts_client_created_us: int = 0
    ts_client_send_us: int = 0


@dataclass(frozen=True, slots=True)
class GripperSample:
    q: float
    reported_torque_nm: float
    status_raw: int


@dataclass(frozen=True, slots=True)
class GripperHostTarget:
    """Host ramp/quantization echo; does not establish TX, ACK or completion."""

    q: float
    torque_cmd_nm: float


@dataclass(frozen=True, slots=True)
class GripperState:
    state_id: int
    gripper_type: int
    side: int
    ts_state_real_us: int = 0
    ts_state_pub_us: int = 0
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    sample: GripperSample | None = None
    sample_valid: bool = False
    host_target: GripperHostTarget | None = None
    host_target_valid: bool = False
    host_target_session_id: int = 0
    host_target_sequence: int = 0
    host_target_cmd_id: int = 0
    output_allowed: bool = False
    inhibit_reason: int = InhibitReason.NOT_READY
    trace_session_id: int = 0
    trace_sequence: int = 0
    trace_cmd_id: int = 0
    ts_cmd_received_us: int = 0
    ts_cmd_accepted_us: int = 0
    ts_cmd_first_output_us: int = 0
    ts_cmd_write_ack_us: int = 0
    ts_cmd_first_feedback_us: int = 0
