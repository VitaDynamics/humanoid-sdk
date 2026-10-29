"""Existing hand protocol enums; no online hand client."""
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
