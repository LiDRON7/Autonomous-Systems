from lidron_navigation.grid import RollingGrid


def test_inflates_and_expires_observations():
    grid = RollingGrid(resolution=1.0, size_m=10.0, inflation_m=1.0, expiry_s=2.0)
    origin = (0.0, 0.0)
    grid.observe([(5.2, 5.2)], origin, now_s=1.0)
    assert (5, 5) in grid.occupied
    assert (4, 5) in grid.occupied
    grid.expire(now_s=3.1)
    assert not grid.occupied


def test_reprojects_observations_when_grid_recenters():
    grid = RollingGrid(resolution=1.0, size_m=10.0, inflation_m=0.1, expiry_s=5.0)
    grid.observe([(5.0, 5.0)], (0.0, 0.0), now_s=1.0)
    assert (5, 5) in grid.occupied
    grid.observe([], (2.0, 2.0), now_s=2.0)
    assert (3, 3) in grid.occupied
    assert (5, 5) not in grid.occupied


def test_world_cell_round_trip_stays_within_resolution():
    grid = RollingGrid(resolution=0.5)
    origin = (-4.0, -4.0)
    point = (1.2, -0.7)
    restored = grid.to_world(grid.to_cell(point, origin), origin)
    assert abs(restored[0] - point[0]) <= grid.resolution / 2
    assert abs(restored[1] - point[1]) <= grid.resolution / 2
