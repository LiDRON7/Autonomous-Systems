import math

import numpy as np
import pytest

from navigation.mapping import depth_points_to_world


@pytest.mark.parametrize('yaw, expected', [
    (0.0, [(13, 18), (13, 22)]),
    (math.pi / 2, [(12, 23), (8, 23)]),
])
def test_flu_left_right_to_north_east(yaw, expected):
    world, count = depth_points_to_world([[3, 2, 0], [3, -2, 0]], (10, 20, -5), yaw, -1.5, 1.5)
    assert count == 2
    np.testing.assert_allclose(world, expected)


def test_height_boundaries_forward_cutoff_and_invalid_values():
    raw = [[2, 0, -1.5], [2, 0, 1.5], [2, 0, -1.501], [2, 0, 1.501],
           [0.2, 0, 0], [-1, 0, 0], [np.nan, 0, 0], [2, np.inf, 0]]
    world, count = depth_points_to_world(raw, (0, 0, 0), 0, -1.5, 1.5)
    assert count == 6
    assert world == [(2, 0), (2, 0)]


@pytest.mark.parametrize('yaw, expected', [(0, [(2.1, -0.3)]), (math.pi / 2, [(0.3, 2.1)])])
def test_mount_translation_rotates_with_vehicle(yaw, expected):
    world, _ = depth_points_to_world([[2, 0, 0]], (0, 0, 0), yaw, -0.1, 0.1, (0.1, 0.3, 0.5))
    np.testing.assert_allclose(world, expected, atol=1e-12)


def test_empty_cloud_and_invalid_configuration():
    assert depth_points_to_world([], (0, 0, 0), 0, -1, 1) == ([], 0)
    with pytest.raises(ValueError):
        depth_points_to_world([], (0, 0, 0), 0, 1, -1)
    with pytest.raises(ValueError):
        depth_points_to_world([], (0, 0, 0), 0, -1, 1, (np.nan, 0, 0))


def test_known_flu_obstacle_was_discarded_by_main_filter():
    # A level obstacle ahead has Z near zero; main mistakenly treats Z as forward.
    raw = np.array([[3.0, 0.0, 0.0]])
    original = (raw[:, 1] >= -1.5) & (raw[:, 1] <= 1.5) & (raw[:, 2] > 0.2)
    assert not original.any()
    assert depth_points_to_world(raw, (0, 0, 0), 0, -1.5, 1.5)[0] == [(3, 0)]
