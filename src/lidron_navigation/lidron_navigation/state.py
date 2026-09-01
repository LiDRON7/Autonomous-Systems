"""Mission-state definitions and safety timing."""

from enum import Enum


class MissionState(str, Enum):
    IDLE = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    TAKEOFF = "TAKEOFF"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    REPLANNING = "REPLANNING"
    APPROACH = "APPROACH"
    LANDING_CHECK = "LANDING_CHECK"
    LANDING = "LANDING"
    COMPLETE = "COMPLETE"
    HOLD = "HOLD"
    ABORT = "ABORT"


def data_is_fresh(now_s: float, stamp_s: float | None, timeout_s: float) -> bool:
    return stamp_s is not None and 0.0 <= now_s - stamp_s <= timeout_s
