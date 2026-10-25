"""Frozen hand description nested in Body/RobotInfo; no online hand client."""
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
