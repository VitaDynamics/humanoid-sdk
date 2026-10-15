"""Public hand wire values; angles rad, speeds rad/s."""
from dataclasses import dataclass
from enum import IntEnum


class HandSide(IntEnum):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2


class HandType(IntEnum):
    UNKNOWN = 0
    BRAINCO_REVO2 = 1
    LINKER_L6 = 2


class InhibitReason(IntEnum):
    NONE = 0
    PASSIVE = 1
    EMERGENCY = 2
    DOMAIN_FAULT = 3
    NOT_READY = 4


@dataclass(frozen=True, slots=True)
class HandJointCmd:
    q: float
    dq: float = 0.0  # speed magnitude; zero uses the robot's configured speed


@dataclass(frozen=True, slots=True)
class HandCmd:
    session_id: int
    sequence: int
    hand_type: int
    side: int
    joints: tuple[HandJointCmd, ...]
    protocol_version: int = 1
    cmd_id: int = 0
    input_state_id: int = 0
    input_state_ts_real_us: int = 0
    ts_client_created_us: int = 0
    ts_client_send_us: int = 0


@dataclass(frozen=True, slots=True)
class HandJointState:
    """Normalized feedback; *_raw are model-specific diagnostics only.

    *_raw 字段仅供诊断，含义随设备型号变化，不应依赖。
    """
    q: float = 0.0
    dq: float = 0.0
    current_ma: float = 0.0
    q_cmd: float = 0.0
    dq_cmd: float = 0.0
    q_raw: int = 0
    dq_raw: int = 0
    current_raw: int = 0
    status_raw: int = 0


@dataclass(frozen=True, slots=True)
class HandState:
    state_id: int
    hand_type: int
    side: int
    joints: tuple[HandJointState, ...]
    ts_state_real_us: int = 0
    ts_state_pub_us: int = 0
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    output_allowed: bool = False
    inhibit_reason: int = 4
    last_cmd_session_id: int = 0
    last_cmd_sequence: int = 0
    last_cmd_id: int = 0
    last_valid_age_us: int = (1 << 64) - 1
    estimated_input_period_us: int = 0
    reject_count: int = 0
    last_reject_code: int = 0
    ingress_drop_count: int = 0
    ts_cmd_received_us: int = 0
    ts_cmd_accepted_us: int = 0
    ts_cmd_first_output_us: int = 0
    ts_cmd_write_ack_us: int = 0
    ts_cmd_first_feedback_us: int = 0
    timeout_count: int = 0
    crc_error_count: int = 0


@dataclass(frozen=True, slots=True)
class FingerTouchMeta:
    ts_request_us: int
    ts_acquired_hw_us: int
    ts_ingested_us: int
    sample_seq: int
    status_flags: int
    direction_flags: int


@dataclass(frozen=True, slots=True)
class HandTouch:
    state_id: int
    hand_type: int
    side: int
    sensor_type: int
    ts_state_pub_us: int
    fingers: tuple[FingerTouchMeta, ...]
    raw_words: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class HandInfo:
    ts_info_us: int
    hand_type: int
    firmware: str
    side: int
    dof: int
    joint_names: tuple[str, ...]
    q_min: tuple[float, ...]
    q_max: tuple[float, ...]
    dq_max: tuple[float, ...]
    touch_available: bool = False
    touch_enabled: bool = False
    touch_fingers: int = 0
    touch_words_per_finger: int = 0
    ts_state_pub_us: int = 0
