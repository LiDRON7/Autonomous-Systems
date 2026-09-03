import numpy as np
from perception.depth import minimum_depth


def test_detects_central_obstacle():
    image = np.full((20, 20), 5.0, dtype=np.float32)
    image[9:11, 9:11] = 1.0
    result = minimum_depth(image, threshold_m=1.5)
    assert result.detected
    assert result.distance_m == 1.0


def test_ignores_invalid_depth():
    image = np.zeros((10, 10), dtype=np.float32)
    image[5, 5] = np.nan
    result = minimum_depth(image, threshold_m=1.5)
    assert not result.detected
    assert np.isinf(result.distance_m)


def test_ignores_obstacle_outside_central_roi():
    image = np.full((20, 20), 5.0, dtype=np.float32)
    image[0, 0] = 0.5
    result = minimum_depth(image, threshold_m=1.5, roi_ratio=0.4)
    assert not result.detected
    assert result.distance_m == 5.0


def test_clamps_invalid_roi_ratio():
    image = np.full((10, 10), 2.0, dtype=np.float32)
    assert minimum_depth(image, threshold_m=3.0, roi_ratio=2.0).detected


def test_rejects_non_image_input():
    result = minimum_depth(np.array([1.0, 2.0]), threshold_m=1.5)
    assert not result.detected
    assert np.isinf(result.distance_m)
