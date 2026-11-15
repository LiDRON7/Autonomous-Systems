"""Provisional FLU cloud mapping for a body-aligned camera that may be pitched down.

Height bounds are relative to the sensor, after levelling by its pitch and before translation;
the min_range forward cutoff is along the sensor's own axis. Output XY is local NED (north/east).
The drone's own roll and pitch are removed too: a hexacopter leans several degrees to fly, and without this the
ground ahead came out as a wall of obstacles in the simulator. Latest attitude is used, not timestamp-aligned.
"""

import math
import numpy as np


def depth_points_to_world(raw, position, yaw, min_height, max_height,
                          sensor_translation=(0.0, 0.0, 0.0), pitch_down=0.0, min_range=0.2, roll=0.0, pitch=0.0):
    """Return retained world XY points and finite input count.

    Translation is sensor origin in body FLU meters. pitch_down (rad) is how far the camera
    looks below the body's forward axis; roll and yaw of the mount must be zero.
    Vertical translation does not change the sensor-relative height band.
    Points closer than min_range (m) are dropped: set it past the drone's own parts in view (prop guards).
    """
    if min_height > max_height:
        raise ValueError("minimum height must not exceed maximum height")
    translation = np.asarray(sensor_translation, dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("sensor translation must contain three finite values")
    points = np.asarray(raw, dtype=float).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    valid_count = len(points)
    points = points[points[:, 0] > min_range]
    cp, sp = math.cos(pitch_down), math.sin(pitch_down)
    forward = points[:, 0] * cp + points[:, 2] * sp
    up = points[:, 2] * cp - points[:, 0] * sp
    left = points[:, 1]
    # Level the body: roll (PX4, right side down positive), then pitch (nose up positive).
    cr, sr, cb, sb = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    left, up = left * cr - up * sr, left * sr + up * cr
    forward, up = forward * cb - up * sb, forward * sb + up * cb
    keep = (up >= min_height) & (up <= max_height)
    forward = forward[keep] + translation[0]
    left = left[keep] + translation[1]
    c, s = math.cos(yaw), math.sin(yaw)
    north = position[0] + c * forward + s * left
    east = position[1] + s * forward - c * left
    return list(zip(north.tolist(), east.tolist())), valid_count
