"""Original PR 79 grid/planner checks, now collected by pytest."""

from navigation.grid import RollingGrid
from navigation.planner import astar


def isolated_tests():
    checks = {}
    g = RollingGrid(resolution=0.5, size_m=30.0, inflation_m=1.0, expiry_s=2.0)
    origin = (0.0, 0.0)
    g.observe([(5.0, 5.0)], origin, 10.0)
    center = g.to_cell((5.0, 5.0), origin)
    checks['grid_insertion'] = center in g.occupied
    # A 1 m Euclidean-radius inflation at 0.5 m resolution has 13 cell offsets.
    checks['inflation'] = len(g.occupied) == 13
    initial = set(g.occupied)
    g.observe([], origin, 11.5)
    checks['persist_before_expiry'] = set(g.occupied) == initial
    g.observe([], origin, 12.1)
    checks['expire_after_timeout'] = not g.occupied
    g.observe([(5.0, 5.0)], origin, 20.0)
    g.observe([(5.0, 5.0)], origin, 21.0)
    g.observe([], origin, 22.5)
    checks['refresh_extends_lifetime'] = bool(g.occupied)
    # Check that the real A* function treats occupied cells as blocked.
    obstacles = {(6, y) for y in range(3, 9)}
    try:
        route = astar((2, 5), (11, 5), obstacles, 20)
        checks['astar_uses_blocked_cells'] = bool(route) and not any(
            tuple(c) in obstacles for c in route
        )
    except Exception as exc:
        checks['astar_uses_blocked_cells'] = False
        checks['astar_exception'] = str(exc)
    return checks


def test_isolated_grid_and_planner_checks():
    checks = isolated_tests()
    assert all(value is True for value in checks.values()), checks
