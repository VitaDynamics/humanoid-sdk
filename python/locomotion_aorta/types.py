from dataclasses import dataclass
from enum import Enum


PROTOCOL_VERSION = 1


class ControlOperation(str, Enum):
    PREPARE = "PREPARE"
    COMMIT = "COMMIT"
    CANCEL = "CANCEL"


class ExternalPhase(str, Enum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    ARMED = "ARMED"
    TRANSITIONING = "TRANSITIONING"
    RUNNING = "RUNNING"
    TIMEOUT_HOLD = "TIMEOUT_HOLD"
    CANCELLED = "CANCELLED"
    EXPIRED = "EXPIRED"
    EMERGENCY = "EMERGENCY"


@dataclass(frozen=True, slots=True)
class MotorCommand:
    mode: int = 0
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


@dataclass(frozen=True, slots=True)
class LowCmd:
    cmd_id: int
    motor_cmds: tuple[MotorCommand, ...]
    external_version: int = 0
    external_session_id: int = 0
    external_sequence: int = 0


@dataclass(frozen=True, slots=True)
class MotorState:
    q: float = 0.0
    dq: float = 0.0
    tau_est: float = 0.0
    temperature: float = 0.0


@dataclass(frozen=True, slots=True)
class HumanoidLowState:
    state_id: int
    stamp_ns: int
    motor_states: tuple[MotorState, ...]
    attitude_valid: bool = True


@dataclass(frozen=True, slots=True)
class ControlRequest:
    request_id: str
    client_id: str
    operation: ControlOperation
    action_name: str = "EXTERNAL"
    lease_id: str = ""
    requested_lease_ms: int = 0
    protocol_version: int = PROTOCOL_VERSION


@dataclass(frozen=True, slots=True)
class ControlAck:
    request_id: str
    operation: ControlOperation
    accepted: bool
    error_code: int = 0
    reason: str = ""
    lease_id: str = ""
    lease_deadline_monotonic_ns: int = 0
    protocol_version: int = PROTOCOL_VERSION


@dataclass(frozen=True, slots=True)
class ControlStatus:
    status_seq: int
    current_action: str
    phase: ExternalPhase
    client_id: str = ""
    lease_id: str = ""
    lease_deadline_monotonic_ns: int = 0
    generation: int = 0
    last_accepted_cmd_id: int = 0
    last_valid_age_ms: int = 0
    estimated_input_period_us: int = 0
    timeout_hold: bool = False
    last_reject_code: int = 0
    reject_count: int = 0
    ingress_drop_count: int = 0
    stamp_monotonic_ns: int = 0
    protocol_version: int = PROTOCOL_VERSION
