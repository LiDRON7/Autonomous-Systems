import pytest
from navigation.planner import astar, landing_search_offset, simplify


def test_routes_around_wall():
    wall = {(4, y) for y in range(8)}
    route = astar((1, 1), (8, 1), wall, width=10)
    assert route[0] == (1, 1)
    assert route[-1] == (8, 1)
    assert not set(route) & wall


def test_reports_unreachable_goal():
    occupied = {(x, 2) for x in range(5)}
    assert astar((1, 1), (1, 3), occupied, width=5) == []


def test_simplifies_clear_route():
    route = [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert simplify(route, set()) == [(0, 0), (3, 0)]


def test_rejects_occupied_start_or_goal():
    assert astar((1, 1), (3, 3), {(1, 1)}, width=5) == []
    assert astar((1, 1), (3, 3), {(3, 3)}, width=5) == []


def test_landing_search_expands_after_first_ring():
    assert landing_search_offset(0, 1.5) == (1.5, 0.0)
    assert landing_search_offset(8, 1.5) == (3.0, 0.0)


def test_landing_search_rejects_invalid_values():
    with pytest.raises(ValueError):
        landing_search_offset(-1, 1.0)
    with pytest.raises(ValueError):
        landing_search_offset(0, 0.0)
