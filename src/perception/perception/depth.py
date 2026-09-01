"""Depth-image safety calculations independent of ROS."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DepthResult:
    distance_m: float
    detected: bool


def minimum_depth(
    image: np.ndarray,
    threshold_m: float,
    roi_ratio: float = 0.4,
) -> DepthResult:
    """Return the nearest valid depth in the image's central region."""
    if image.ndim != 2 or image.size == 0:
        return DepthResult(float("inf"), False)
    ratio = min(1.0, max(0.05, roi_ratio))
    height, width = image.shape
    roi_h, roi_w = max(1, int(height * ratio)), max(1, int(width * ratio))
    top, left = (height - roi_h) // 2, (width - roi_w) // 2
    region = image[top : top + roi_h, left : left + roi_w]
    valid = region[np.isfinite(region) & (region > 0.0)]
    distance = float(valid.min()) if valid.size else float("inf")
    return DepthResult(distance, distance < threshold_m)
