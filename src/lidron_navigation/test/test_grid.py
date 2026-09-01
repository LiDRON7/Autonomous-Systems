from lidron_navigation.grid import RollingGrid


def test_inflates_and_expires_observations():
    grid = RollingGrid(resolution=1.0, size_m=10.0, inflation_m=1.0, expiry_s=2.0)
    origin = (0.0, 0.0)
    grid.observe([(5.2, 5.2)], origin, now_s=1.0)
    assert (5, 5) in grid.occupied
    assert (4, 5) in grid.occupied
    grid.expire(now_s=3.1)
    assert not grid.occupied
