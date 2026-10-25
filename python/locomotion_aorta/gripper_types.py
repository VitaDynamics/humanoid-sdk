"""Frozen gripper description nested in Body/RobotInfo (metres and N·m)."""
from dataclasses import dataclass
from lowlevel.GripperLimitSource import GripperLimitSource
from lowlevel.GripperType import GripperType


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
