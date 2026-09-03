"""Conservative landing-zone assessment from a downward point cloud."""

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class LandingLimits:
    footprint_m: float = 1.2
    min_points: int = 80
    max_slope_deg: float = 8.0
    max_roughness_m: float = 0.08
    obstacle_height_m: float = 0.20
    clearance_m: float = 0.75


@dataclass(frozen=True)
class LandingAssessment:
    suitable: bool
    reason: str
    point_count: int
    slope_deg: float = float("inf")
    roughness_m: float = float("inf")
    clear_radius_m: float = -1.0

    def as_dict(self) -> dict:
        return {
            key: (None if isinstance(value, float) and not np.isfinite(value) else value)
            for key, value in asdict(self).items()
        }


def _reject(reason: str, count: int) -> LandingAssessment:
    return LandingAssessment(False, reason, count)


def assess_landing_zone(
    points: np.ndarray,
    limits: LandingLimits | None = None,
) -> LandingAssessment:
    """Fit a local ground plane and validate slope, roughness, and clearance."""
    limits = limits or LandingLimits()
    cloud = np.asarray(points, dtype=float).reshape((-1, 3))
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    half = limits.footprint_m / 2.0
    footprint = cloud[(np.abs(cloud[:, 0]) <= half) & (np.abs(cloud[:, 1]) <= half)]
    count = len(footprint)
    if count < limits.min_points:
        return _reject("insufficient_ground_points", count)

    center = footprint.mean(axis=0)
    _, _, vh = np.linalg.svd(footprint - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    slope = float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0))))
    residuals = np.abs((footprint - center) @ normal)
    roughness = float(np.percentile(residuals, 95))
    if slope > limits.max_slope_deg:
        return LandingAssessment(False, "slope_too_high", count, slope, roughness)
    if roughness > limits.max_roughness_m:
        return LandingAssessment(False, "surface_too_rough", count, slope, roughness)

    heights = (cloud - center) @ normal
    raised = cloud[heights > limits.obstacle_height_m]
    clear_radius = -1.0
    if len(raised):
        clear_radius = float(np.min(np.linalg.norm(raised[:, :2], axis=1)))
        if clear_radius < limits.clearance_m:
            return LandingAssessment(
                False, "obstacle_inside_clearance", count, slope, roughness, clear_radius
            )
    return LandingAssessment(True, "safe", count, slope, roughness, clear_radius)
