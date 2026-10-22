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


@pytest.mark.parametrize('start,goal,width', [((-1, 0), (2, 2), 5), ((0, 0), (5, 0), 5), ((0, 0), (0, 0), 0)])
def test_rejects_out_of_bounds(start, goal, width):
    assert astar(start, goal, set(), width) == []


def test_cannot_squeeze_between_diagonal_obstacles():
    assert astar((0, 0), (1, 1), {(1, 0), (0, 1)}, 2) == []


def test_simplification_checks_cells_between_rounded_samples():
    from navigation.planner import _line_clear
    assert not _line_clear((0, 0), (2, 1), {(1, 0)})
    assert not _line_clear((2, 1), (0, 0), {(1, 0)})


def test_simplification_preserves_safe_detour():
    route = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 1), (2, 0)]
    result = simplify(route, {(1, 0), (1, 1)})
    assert result[0] == route[0] and result[-1] == route[-1]
    assert len(result) > 2


def test_simplification_rejects_unsafe_input_segment():
    assert simplify([(0, 0), (1, 1)], {(1, 0)}) == []
