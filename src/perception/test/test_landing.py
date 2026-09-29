import json

import numpy as np
from perception.filters import preprocess_landing_cloud
from perception.landing import LandingLimits, assess_landing_zone, evaluate_landing_zone

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


def test_refines_only_ground_inliers_but_keeps_obstacles():
    obstacles = np.array([(0.3, y, 0.7) for y in np.linspace(-0.3, 0.3, 20)])
    points = np.vstack([flat_plane(), obstacles])
    result = evaluate_landing_zone(points, LandingLimits(ransac_dist_threshold=0.03))
    assert result.assessment.reason == "obstacle_inside_clearance"
    assert result.assessment.slope_deg < 0.01
    assert result.assessment.roughness_m < 0.001
    np.testing.assert_allclose(result.ground_points, flat_plane())
    np.testing.assert_allclose(result.non_ground_points, obstacles)


def test_selects_footprint_before_ransac_and_preserves_outside_points():
    outside = np.array([(1.0, y, 1.0) for y in np.linspace(-2, 2, 200)])
    nearby = np.array([[0.65, 0.0, 0.5]])
    result = evaluate_landing_zone(np.vstack([flat_plane(), outside, nearby]), LIMITS)
    assert result.assessment.reason == "obstacle_inside_clearance"
    assert result.assessment.point_count == 100
    assert result.assessment.clear_radius_m == 0.65
    np.testing.assert_allclose(result.ground_points, flat_plane())
    np.testing.assert_allclose(result.non_ground_points, np.vstack([outside, nearby]))


def test_rejects_degenerate_candidate_and_preserves_points():
    points = np.zeros((100, 3))
    result = evaluate_landing_zone(points, LIMITS)
    assert result.assessment.reason == "ground_plane_not_found"
    assert not result.assessment.suitable
    assert result.ground_points.shape == (0, 3)
    np.testing.assert_array_equal(result.non_ground_points, points)


def test_ransac_failure_assessment_is_valid_json():
    result = assess_landing_zone(flat_plane(), LandingLimits(ransac_num_iterations=0))
    assert result.reason == "ground_plane_not_found"
    assert json.loads(json.dumps(result.as_dict(), allow_nan=False))["slope_deg"] is None


def test_roughness_uses_only_ground_inliers():
    # Low outliers do not count as ground roughness or raised obstacles.
    points = np.vstack([flat_plane(), [[0.2, y, -0.5] for y in np.linspace(-0.4, 0.4, 20)]])
    result = assess_landing_zone(points, LandingLimits(ransac_dist_threshold=0.03))
    assert result.suitable
    assert result.roughness_m < 0.001


def test_preprocessing_and_ransac_keep_raised_cluster_for_clearance():
    axis = np.linspace(-0.5, 0.5, 20)
    ground = np.array([(x, y, 0.0) for x in axis for y in axis])
    cluster_axis = np.linspace(0.1, 0.3, 6)
    raised = np.array([(x, y, 0.5) for x in cluster_axis for y in cluster_axis])
    filtered = preprocess_landing_cloud(np.vstack([ground, raised]))
    result = evaluate_landing_zone(filtered)
    assert result.assessment.reason == "obstacle_inside_clearance"
    assert len(result.ground_points) >= 80
    assert len(result.non_ground_points) > 0
    assert len(result.ground_points) + len(result.non_ground_points) == len(filtered)


def test_noisy_sloped_ground_is_refined_with_svd():
    points = flat_plane()
    points[:, 2] = 0.05 * points[:, 0]
    points[:, 2] += np.random.default_rng(2).normal(0.0, 0.004, len(points))
    result = assess_landing_zone(points, LandingLimits(ransac_dist_threshold=0.03))
    assert result.suitable
    assert abs(result.slope_deg - np.degrees(np.arctan(0.05))) < 0.5
    assert 0.003 < result.roughness_m < 0.015
