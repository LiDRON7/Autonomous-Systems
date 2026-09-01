import numpy as np
from perception.landing import LandingLimits, assess_landing_zone

LIMITS = LandingLimits(min_points=25)


def flat_plane(z=0.0):
    axis = np.linspace(-0.5, 0.5, 10)
    return np.array([(x, y, z) for x in axis for y in axis])


def test_accepts_flat_clear_surface():
    result = assess_landing_zone(flat_plane(), LIMITS)
    assert result.suitable
    assert result.reason == "safe"


def test_rejects_steep_surface():
    points = flat_plane()
    points[:, 2] = points[:, 0] * 0.5
    result = assess_landing_zone(points, LIMITS)
    assert not result.suitable
    assert result.reason == "slope_too_high"


def test_rejects_nearby_obstacle():
    points = np.vstack([flat_plane(), [0.2, 0.2, 0.5]])
    result = assess_landing_zone(points, LIMITS)
    assert not result.suitable
    assert result.reason == "obstacle_inside_clearance"


def test_rejects_sparse_cloud():
    result = assess_landing_zone(flat_plane()[:10], LIMITS)
    assert not result.suitable
    assert result.reason == "insufficient_ground_points"


def test_rejects_rough_surface():
    points = flat_plane()
    points[::2, 2] = 0.2
    result = assess_landing_zone(points, LIMITS)
    assert not result.suitable
    assert result.reason == "surface_too_rough"


def test_ignores_non_finite_points():
    points = np.vstack([flat_plane(), [np.nan, 0.0, 0.0]])
    result = assess_landing_zone(points, LIMITS)
    assert result.suitable
