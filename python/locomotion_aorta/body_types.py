"""Public body DTOs. Angles rad; robot/client clocks are explicitly microseconds."""
from dataclasses import dataclass
from lowlevel.BodyCmdAction import BodyCmdAction
from lowlevel.BodySampleStatus import BodySampleStatus
from lowlevel.BodyImuLocation import BodyImuLocation
from lowlevel.BodySlotRole import BodySlotRole
from lowlevel.EndEffectorKind import EndEffectorKind
from aorta.services.common.ServiceStatus import ServiceStatus
from .hand_types import HandInfo

BODY_SLOTS = 31
BODY_PROTOCOL_VERSION = 1

@dataclass(frozen=True, slots=True)
class BodyJointCmd:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0

@dataclass(frozen=True, slots=True)
class BodyCmd:
    cmd_id: int
    joints: tuple[BodyJointCmd, ...]
    action: int = BodyCmdAction.EXTERNAL
    protocol_version: int = BODY_PROTOCOL_VERSION
    session_id: int = 0
    sequence: int = 0
    input_state_id: int = 0
    input_state_ts_real_us: int = 0
    ts_client_created_us: int = 0
    ts_client_send_us: int = 0
    root_quat: tuple[float, float, float, float] | None = None

@dataclass(frozen=True, slots=True)
class BodyJointState:
    body_index: int
    populated: bool
    is_mock: bool
    status: int
    valid_fields: int
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    q: float = 0.0
    dq: float = 0.0
    ddq: float = 0.0
    tau_est: float = 0.0
    temperature_deg_c: float = 0.0
    lost: int = 0

@dataclass(frozen=True, slots=True)
class BodyImuState:
    quaternion: tuple[float, ...]
    gyroscope: tuple[float, ...]
    accelerometer: tuple[float, ...]
    rpy: tuple[float, ...]  # degrees, follows the producer's IMU contract

@dataclass(frozen=True, slots=True)
class BodyImuSample:
    location: int
    state: BodyImuState | None
    status: int
    valid_fields: int
    ts_sampled_us: int = 0
    ts_acquired_hw_us: int = 0
    ts_ingested_us: int = 0
    ts_assembled_us: int = 0
    temperature_deg_c: float = 0.0
    device_timestamp_ms: int = 0
    host_timestamp_us: int = 0

@dataclass(frozen=True, slots=True)
class BodyAppliedCmd:
    action: int = BodyCmdAction.UNKNOWN
    session_id: int = 0
    sequence: int = 0
    cmd_id: int = 0
    ts_first_output_us: int = 0
    joints: tuple[BodyJointCmd, ...] = ()
    source_key_valid: bool = False

@dataclass(frozen=True, slots=True)
class BodyState:
    state_id: int
    joints: tuple[BodyJointState, ...]
    protocol_version: int = BODY_PROTOCOL_VERSION
    roster_version: int = 0
    ts_state_real_us: int = 0
    ts_state_pub_us: int = 0
    ts_consumed_us: int = 0
    attitude_valid: bool = False
    imu_states: tuple[BodyImuSample, ...] = ()
    applied_cmd: BodyAppliedCmd | None = None

@dataclass(frozen=True, slots=True)
class BodyJointCapability:
    body_index: int
    slot_role: int
    is_mock: bool
    populated: bool
    supported_fields: int
    valid_limits: int
    q_min: float
    q_max: float
    dq_min: float
    dq_max: float
    tau_min: float
    tau_max: float
    kp_min: float
    kp_max: float
    kd_min: float
    kd_max: float

@dataclass(frozen=True, slots=True)
class BodyMotorIdentity:
    bus: str
    node_id: int
    present: bool

@dataclass(frozen=True, slots=True)
class BodyInfo:
    roster_version: int
    slot_count: int
    joint_names: tuple[str, ...]
    joints: tuple[BodyJointCapability, ...]
    motor_identities: tuple[BodyMotorIdentity, ...]

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

@dataclass(frozen=True, slots=True)
class EndEffectorCapability:
    side: int
    kind: int
    configured: bool
    is_mock: bool
    get_info_service: str
    hand: HandInfo | None
    gripper: GripperInfo | None
    ready: bool
    status: int
    reason: str

@dataclass(frozen=True, slots=True)
class RobotInfo:
    protocol_version: int
    ts_info_us: int
    body: BodyInfo | None
    end_effectors: tuple[EndEffectorCapability, ...]

@dataclass(frozen=True, slots=True)
class GetRobotInfoResponse:
    status: int
    info: RobotInfo | None
    error_message: str
