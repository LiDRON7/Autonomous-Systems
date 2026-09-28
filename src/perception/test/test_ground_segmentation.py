import numpy as np
import pytest

from perception.ground_segmentation import ransac_ground_inliers


def plane():
    axis = np.linspace(-0.5, 0.5, 10)
    return np.array([(x, y, 0.0) for x in axis for y in axis])


def test_separates_ground_from_raised_points():
    points = np.vstack([plane(), [[0.1, 0.1, 0.5], [0.2, 0.2, 0.8]]])
    mask = ransac_ground_inliers(points, 0.03, 100, 80)
    assert mask is not None
    assert mask[:100].all()
    assert not mask[100:].any()


@pytest.mark.parametrize("points", [np.empty((0, 3)), np.zeros((100, 3)),
                                   np.array([(x, 0, 0) for x in range(100)])])
def test_rejects_clouds_without_a_plane(points):
    assert ransac_ground_inliers(points, 0.03, 100, 3) is None


def test_requires_enough_inliers():
    points = np.random.default_rng(5).uniform(-0.5, 0.5, (100, 3))
    assert ransac_ground_inliers(points, 0.001, 100, 80) is None


@pytest.mark.parametrize("distance,iterations", [(0, 100), (-1, 100),
                                                (float("nan"), 100),
                                                (float("inf"), 100), (0.03, 0)])
def test_invalid_settings_fail_without_raising(distance, iterations):
    assert ransac_ground_inliers(plane(), distance, iterations, 80) is None


def test_segmentation_is_repeatable():
    points = plane()
    points[:, 2] += np.random.default_rng(1).normal(0, 0.005, 100)
    first = ransac_ground_inliers(points, 0.03, 100, 80)
    assert first is not None
    np.testing.assert_array_equal(first, ransac_ground_inliers(points, 0.03, 100, 80))
