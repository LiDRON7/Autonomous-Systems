from lidron_navigation.state import MissionState, can_transition, data_is_fresh


def test_nominal_mission_transitions_are_allowed():
    route = [
        MissionState.IDLE,
        MissionState.PREFLIGHT,
        MissionState.TAKEOFF,
        MissionState.PLANNING,
        MissionState.NAVIGATING,
        MissionState.APPROACH,
        MissionState.LANDING_CHECK,
        MissionState.LANDING,
        MissionState.COMPLETE,
    ]
    assert all(can_transition(first, second) for first, second in zip(route, route[1:]))


def test_unsafe_transition_is_rejected():
    assert not can_transition(MissionState.IDLE, MissionState.LANDING)


def test_data_freshness_handles_missing_and_future_samples():
    assert data_is_fresh(10.0, 9.5, 1.0)
    assert not data_is_fresh(10.0, None, 1.0)
    assert not data_is_fresh(10.0, 10.1, 1.0)
