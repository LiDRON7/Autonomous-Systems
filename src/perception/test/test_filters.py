import numpy as np
from perception.filters import (
    finite_points,
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
