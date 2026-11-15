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


def test_camera_pitched_down_does_not_turn_far_ground_into_an_obstacle():
    # 5 m up, camera 10 deg down: the ground 25 m ahead sits inside the camera's own +-1.5 m band.
    pitch = math.radians(10)
    ground = [25 * math.cos(pitch) - 5 * math.sin(pitch), 0.0, -5 * math.cos(pitch) + 25 * math.sin(pitch)]
    assert depth_points_to_world([ground], (0, 0, -5), 0, -1.5, 1.5)[0]            # old: false obstacle
    assert not depth_points_to_world([ground], (0, 0, -5), 0, -1.5, 1.5, pitch_down=pitch)[0]
    # an obstacle at the drone's own height, 4 m ahead, is still kept and lands 4 m north
    level = [4 * math.cos(pitch), 0.0, 4 * math.sin(pitch)]
    world, _ = depth_points_to_world([level], (0, 0, -5), 0, -1.5, 1.5, pitch_down=pitch)
    np.testing.assert_allclose(world, [(4, 0)], atol=1e-9)


def test_min_range_drops_the_drones_own_guards():
    raw = [[0.3, 0.2, 0.05], [0.3, -0.2, 0.05], [4.0, 0.0, 0.0]]   # two guard hits and a real obstacle
    assert len(depth_points_to_world(raw, (0, 0, 0), 0, -1.5, 1.5)[0]) == 3
    world, count = depth_points_to_world(raw, (0, 0, 0), 0, -1.5, 1.5, min_range=0.6)
    assert count == 3 and world == [(4.0, 0.0)]


def test_drone_leaning_forward_does_not_turn_the_ground_ahead_into_a_wall():
    # Seen in the sim: flying at 2.2 m/s the drone leans nose down and ~1100 ground cells filled the map.
    mount, lean, h = math.radians(10), math.radians(-12), 2.5   # PX4 pitch: nose up positive
    total = mount - lean                                       # camera looks 22 deg below the horizon
    for d in (4.0, 8.0, 15.0):                                 # ground ahead, true height -h below the camera
        ray = math.atan2(h, d) - total                         # angle of that ground point in the camera frame
        r = math.hypot(d, h)
        ground = [r * math.cos(ray), 0.0, -r * math.sin(ray)]
        if d >= 8:   # before: ground 8 m and more ahead became obstacles
            assert depth_points_to_world([ground], (0, 0, -h), 0, -1.5, 1.5, pitch_down=mount)[0]
        assert not depth_points_to_world([ground], (0, 0, -h), 0, -1.5, 1.5, pitch_down=mount, pitch=lean)[0]
    # a pole at the drone's height 5 m ahead is still kept, 5 m north
    pole = [5 * math.cos(total), 0.0, 5 * math.sin(total)]
    world, _ = depth_points_to_world([pole], (0, 0, -h), 0, -1.5, 1.5, pitch_down=mount, pitch=lean)
    np.testing.assert_allclose(world, [(5, 0)], atol=1e-9)


def test_rolled_drone_keeps_a_level_obstacle_level():
    roll = math.radians(15)                                    # right side down
    side = [4.0, 2 * math.cos(roll), -2 * math.sin(roll)]      # point 2 m left at camera height, seen rolled
    world, _ = depth_points_to_world([side], (0, 0, -5), 0, -0.1, 0.1, roll=roll)
    np.testing.assert_allclose(world, [(4, -2)], atol=1e-9)
