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


ALLOWED_TRANSITIONS = {
    MissionState.IDLE: {MissionState.PREFLIGHT, MissionState.HOLD},
    MissionState.PREFLIGHT: {MissionState.TAKEOFF, MissionState.HOLD, MissionState.ABORT},
    MissionState.TAKEOFF: {MissionState.PLANNING, MissionState.HOLD, MissionState.ABORT},
    MissionState.PLANNING: {MissionState.NAVIGATING, MissionState.HOLD, MissionState.ABORT},
    MissionState.NAVIGATING: {
        MissionState.PLANNING,
        MissionState.REPLANNING,
        MissionState.APPROACH,
        MissionState.HOLD,
        MissionState.ABORT,
    },
    MissionState.REPLANNING: {MissionState.NAVIGATING, MissionState.HOLD, MissionState.ABORT},
    MissionState.APPROACH: {MissionState.LANDING_CHECK, MissionState.HOLD, MissionState.ABORT},
    MissionState.LANDING_CHECK: {
        MissionState.LANDING,
        MissionState.PLANNING,
        MissionState.HOLD,
        MissionState.ABORT,
    },
    MissionState.LANDING: {MissionState.COMPLETE, MissionState.ABORT},
    MissionState.HOLD: {MissionState.REPLANNING, MissionState.ABORT},
    MissionState.COMPLETE: {MissionState.PREFLIGHT},
    MissionState.ABORT: {MissionState.PREFLIGHT},
}


def can_transition(current: MissionState, target: MissionState) -> bool:
    return current == target or target in ALLOWED_TRANSITIONS[current]


def data_is_fresh(now_s: float, stamp_s: float | None, timeout_s: float) -> bool:
    return stamp_s is not None and 0.0 <= now_s - stamp_s <= timeout_s
