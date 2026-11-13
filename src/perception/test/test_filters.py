import numpy as np
import pytest
from perception.filters import (
    PassThroughBounds,
    exclude_box,
    finite_points,
    lidar_to_landing_frame,
    passthrough_filter,
    preprocess_landing_cloud,
    statistical_outlier_removal,
    voxel_downsample,
)


def test_finite_filter_removes_invalid_rows():
    points = np.array([[0, 0, 0], [np.nan, 1, 1], [1, np.inf, 1]])
    assert finite_points(points).tolist() == [[0.0, 0.0, 0.0]]


def test_voxel_filter_keeps_one_point_per_cell():
    points = np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [0.06, 0, 0]])
    result = voxel_downsample(points, voxel_size_m=0.03)
    assert len(result) == 2


def test_kdtree_sor_removes_isolated_point():
    rng = np.random.default_rng(7)
    cluster = rng.normal(0.0, 0.02, size=(100, 3))
    points = np.vstack([cluster, [10.0, 10.0, 10.0]])
    result = statistical_outlier_removal(points, mean_k=10, threshold=2.0)
    assert len(result) == 100
    assert not np.any(np.all(result == [10.0, 10.0, 10.0], axis=1))


def test_small_cloud_is_returned_without_sor():
    points = np.array([[0, 0, 0], [1, 1, 1]], dtype=float)
    assert np.array_equal(statistical_outlier_removal(points, mean_k=5), points)


def test_pipeline_applies_voxel_and_sor_filters():
    rng = np.random.default_rng(9)
    cluster = rng.normal(0.0, 0.005, size=(100, 3))
    points = np.vstack([cluster, [4.0, 4.0, 4.0]])
    result = preprocess_landing_cloud(points, voxel_size_m=0.03, mean_k=3, threshold=1.0)
    assert len(result) < len(points)


def test_passthrough_uses_inclusive_xyz_bounds():
    bounds = PassThroughBounds(-1, 1, -2, 2, -3, 3)
    points = np.array([
        [-1, -2, -3], [1, 2, 3], [0, 0, 0],
        [-1.1, 0, 0], [1.1, 0, 0], [0, -2.1, 0],
        [0, 2.1, 0], [0, 0, -3.1], [0, 0, 3.1], [np.nan, 0, 0],
    ])
    np.testing.assert_array_equal(passthrough_filter(points, bounds), points[:3])


def test_passthrough_defaults_do_not_crop_ground_by_height():
    points = np.array([[0, 0, -50], [0, 0, 50], [11, 0, 0]])
    np.testing.assert_array_equal(passthrough_filter(points), points[:2])


def test_exclusion_removes_only_points_inside_inclusive_box():
    bounds = PassThroughBounds(-0.2, 0.2, -0.2, 0.2, -0.08, -0.02)
    points = np.array([
        [-0.2, -0.2, -0.08], [0.2, 0.2, -0.02], [0.0, 0.0, -0.05],
        [0.21, 0.0, -0.05], [0.0, 0.0, 0.0], [np.nan, 0.0, 0.0],
    ])
    np.testing.assert_allclose(exclude_box(points, bounds), points[3:5])


def test_exclusion_is_disabled_without_bounds():
    points = np.array([[0.0, 0.0, -0.05], [0.5, 0.5, 0.5]])
    np.testing.assert_allclose(exclude_box(points), points)


@pytest.mark.parametrize("points", [[], [[20, 0, 0]], [[np.inf, 0, 0]]])
def test_passthrough_empty_result_has_xyz_shape(points):
    assert passthrough_filter(np.array(points)).shape == (0, 3)


@pytest.mark.parametrize("kwargs", [
    {"x_min": 2, "x_max": 1}, {"y_min": float("nan")},
    {"z_min": 2, "z_max": 1}, {"z_min": float("inf")},
])
def test_passthrough_rejects_invalid_bounds(kwargs):
    with pytest.raises(ValueError):
        PassThroughBounds(**kwargs)


def test_roi_must_cover_footprint_and_clearance():
    with pytest.raises(ValueError):
        PassThroughBounds(x_min=-0.6).validate_landing_area(1.2, 0.75)
    with pytest.raises(ValueError):
        PassThroughBounds(y_max=0.9).validate_landing_area(2.0, 0.75)
    PassThroughBounds(-1, 1, -1, 1).validate_landing_area(1.5, 1.0)


def test_pipeline_crops_before_voxel_selection():
    # Both points share a voxel; cropping first must retain the inside point.
    points = np.array([[0.051, 0, 0], [0.049, 0, 0]])
    result = preprocess_landing_cloud(
        points, voxel_size_m=0.1, roi=PassThroughBounds(x_max=0.05),
    )
    np.testing.assert_allclose(result, [[0.049, 0, 0]])


def test_pipeline_excludes_self_returns_before_voxel_selection():
    # Both points share a voxel; self-exclusion must preserve the external point.
    points = np.array([[0.01, 0.01, -0.05], [0.01, 0.01, 0.01]])
    result = preprocess_landing_cloud(
        points,
        voxel_size_m=0.1,
        exclusion=PassThroughBounds(-0.2, 0.2, -0.2, 0.2, -0.08, -0.02),
    )
    np.testing.assert_allclose(result, [[0.01, 0.01, 0.01]])


def test_roi_retains_clearance_obstacle_outside_landing_footprint():
    from perception.landing import evaluate_landing_zone

    axis = np.linspace(-0.5, 0.5, 20)
    ground = np.array([(x, y, 0) for x in axis for y in axis])
    cloud = np.vstack([ground, [0.65, 0, -0.5], [20, 0, 0]])
    filtered = preprocess_landing_cloud(cloud, mean_k=1000, roi=PassThroughBounds())
    result = evaluate_landing_zone(filtered)
    assert result.assessment.reason == "obstacle_inside_clearance"
    np.testing.assert_allclose(result.non_ground_points, [[0.65, 0, -0.5]])


def _rotation(rpy_deg):
    r, p, y = np.radians(rpy_deg)
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def test_upside_down_lidar_frame_is_already_the_landing_frame():
    points = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, 4.0]], dtype=np.float32)
    np.testing.assert_allclose(lidar_to_landing_frame(points), points, atol=1e-6)


def test_side_mounted_lidar_ground_becomes_a_level_floor_below():
    # Huntsman: Puck LITE on its side (roll 90, pitch 135). Flat ground 3 m under the drone, body x forward, y left, z up.
    from perception.landing import assess_landing_zone
    mount = (90.0, 135.0, 0.0)
    xs, ys = np.meshgrid(np.linspace(-1.5, 1.5, 31), np.linspace(-1.5, 1.5, 31))
    body = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, -3.0)))
    in_lidar_frame = body @ _rotation(mount)            # what the sensor reports
    assert not assess_landing_zone(in_lidar_frame).suitable   # read raw, the floor looks like a wall
    landing = lidar_to_landing_frame(in_lidar_frame, mount)
    np.testing.assert_allclose(landing[:, 2], 3.0, atol=1e-5)          # z down
    np.testing.assert_allclose(landing[:, 0], body[:, 0], atol=1e-5)   # x still forward
    np.testing.assert_allclose(landing[:, 1], -body[:, 1], atol=1e-5)  # y right
    assert assess_landing_zone(landing).suitable
