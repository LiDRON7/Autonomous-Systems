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
    PILOT_OVERRIDE = "PILOT_OVERRIDE"


ALLOWED_TRANSITIONS = {
    MissionState.IDLE: {MissionState.PREFLIGHT, MissionState.HOLD},
    MissionState.PREFLIGHT: {
        MissionState.TAKEOFF, MissionState.HOLD, MissionState.ABORT, MissionState.PILOT_OVERRIDE,
    },
    MissionState.TAKEOFF: {
        MissionState.PLANNING, MissionState.HOLD, MissionState.ABORT, MissionState.PILOT_OVERRIDE,
    },
    MissionState.PLANNING: {
        MissionState.NAVIGATING, MissionState.HOLD, MissionState.ABORT, MissionState.PILOT_OVERRIDE,
    },
    MissionState.NAVIGATING: {
        MissionState.PLANNING,
        MissionState.REPLANNING,
        MissionState.APPROACH,
        MissionState.HOLD,
        MissionState.ABORT,
        MissionState.PILOT_OVERRIDE,
    },
    MissionState.REPLANNING: {MissionState.NAVIGATING, MissionState.HOLD, MissionState.ABORT, MissionState.PILOT_OVERRIDE },
    MissionState.APPROACH: {MissionState.LANDING_CHECK, MissionState.HOLD, MissionState.ABORT, MissionState.PILOT_OVERRIDE},
    MissionState.LANDING_CHECK: {
        MissionState.LANDING,
        MissionState.PLANNING,
        MissionState.HOLD,
        MissionState.ABORT,
        MissionState.PILOT_OVERRIDE,
    },
    MissionState.LANDING: {MissionState.COMPLETE, MissionState.ABORT, MissionState.PILOT_OVERRIDE },
    MissionState.HOLD: {MissionState.REPLANNING, MissionState.ABORT, MissionState.PILOT_OVERRIDE },
    MissionState.COMPLETE: {MissionState.PREFLIGHT},
    MissionState.ABORT: {MissionState.PREFLIGHT},
    MissionState.PILOT_OVERRIDE: {MissionState.PREFLIGHT},
}


def can_transition(current: MissionState, target: MissionState) -> bool:
    return current == target or target in ALLOWED_TRANSITIONS[current]


def data_is_fresh(now_s: float, stamp_s: float | None, timeout_s: float) -> bool:
    return stamp_s is not None and 0.0 <= now_s - stamp_s <= timeout_s
