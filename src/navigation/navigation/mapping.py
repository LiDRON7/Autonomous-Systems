"""Provisional FLU cloud mapping for a level, body-aligned camera.

Height bounds and the 0.2 m forward cutoff are sensor-relative, before translation.
Output XY is local NED (north/east). Latest pose/yaw is used; this is not a
full attitude or timestamp-aligned transform. Verify incoming axes in simulation.
"""

import math
import numpy as np


def depth_points_to_world(raw, position, yaw, min_height, max_height,
                          sensor_translation=(0.0, 0.0, 0.0)):
    """Return retained world XY points and finite input count.

    Translation is sensor origin in body FLU meters; sensor/body axes must align.
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
    points = points[(points[:, 2] >= min_height) &
                    (points[:, 2] <= max_height) & (points[:, 0] > 0.2)]
    body = points + translation
    c, s = math.cos(yaw), math.sin(yaw)
    world = [(position[0] + c * forward + s * left,
              position[1] + s * forward - c * left)
             for forward, left, _up in body]
    return world, valid_count
