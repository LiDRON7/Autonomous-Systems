"""Provisional FLU cloud mapping for a body-aligned camera that may be pitched down.

Height bounds are relative to the sensor, after levelling by its pitch and before translation;
the 0.2 m forward cutoff is along the sensor's own axis. Output XY is local NED (north/east).
Latest pose/yaw is used; this is not a full attitude or timestamp-aligned transform, so the
drone's own pitch and roll in flight are not removed. Verify incoming axes in simulation.
"""

import math
import numpy as np


def depth_points_to_world(raw, position, yaw, min_height, max_height,
                          sensor_translation=(0.0, 0.0, 0.0), pitch_down=0.0):
    """Return retained world XY points and finite input count.

    Translation is sensor origin in body FLU meters. pitch_down (rad) is how far the camera
    looks below the body's forward axis; roll and yaw of the mount must be zero.
    Vertical translation does not change the sensor-relative height band.
    """
    if min_height > max_height:
        raise ValueError("minimum height must not exceed maximum height")
    translation = np.asarray(sensor_translation, dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("sensor translation must contain three finite values")
    points = np.asarray(raw, dtype=float).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    valid_count = len(points)
    points = points[points[:, 0] > 0.2]
    cp, sp = math.cos(pitch_down), math.sin(pitch_down)
    forward = points[:, 0] * cp + points[:, 2] * sp
    up = points[:, 2] * cp - points[:, 0] * sp
    keep = (up >= min_height) & (up <= max_height)
    forward = forward[keep] + translation[0]
    left = points[keep, 1] + translation[1]
    c, s = math.cos(yaw), math.sin(yaw)
    north = position[0] + c * forward + s * left
    east = position[1] + s * forward - c * left
    return list(zip(north.tolist(), east.tolist())), valid_count
