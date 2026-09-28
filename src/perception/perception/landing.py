"""Conservative landing-zone assessment from a downward point cloud."""

from dataclasses import asdict, dataclass

import numpy as np

from .ground_segmentation import ransac_ground_inliers


@dataclass(frozen=True)
class LandingLimits:
    footprint_m: float = 1.2
    min_points: int = 80
    max_slope_deg: float = 8.0
    max_roughness_m: float = 0.08
    obstacle_height_m: float = 0.20
    clearance_m: float = 0.75
    ransac_dist_threshold: float = 0.2
    ransac_num_iterations: int = 100


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


@dataclass(frozen=True)
class LandingEvaluation:
    assessment: LandingAssessment
    ground_points: np.ndarray
    # Includes footprint outliers and all points outside the candidate area.
    non_ground_points: np.ndarray


def assess_landing_zone(
    points: np.ndarray,
    limits: LandingLimits | None = None,
) -> LandingAssessment:
    """Return the assessment while keeping the existing public interface."""
    return evaluate_landing_zone(points, limits).assessment


def evaluate_landing_zone(
    points: np.ndarray,
    limits: LandingLimits | None = None,
) -> LandingEvaluation:
    """Segment the footprint, refine ground with SVD, and check all obstacles."""
    limits = limits or LandingLimits()
    cloud = np.asarray(points, dtype=float).reshape((-1, 3))
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    half = limits.footprint_m / 2.0
    candidate = (np.abs(cloud[:, 0]) <= half) & (np.abs(cloud[:, 1]) <= half)
    footprint = cloud[candidate]
    count = len(footprint)
    ground_mask = np.zeros(len(cloud), dtype=bool)

    def result(assessment: LandingAssessment) -> LandingEvaluation:
        return LandingEvaluation(assessment, cloud[ground_mask], cloud[~ground_mask])

    if count < limits.min_points:
        return result(_reject("insufficient_ground_points", count))

    inliers = ransac_ground_inliers(
        footprint, limits.ransac_dist_threshold, limits.ransac_num_iterations,
        limits.min_points,
    )
    if inliers is None:
        return result(_reject("ground_plane_not_found", count))

    ground = footprint[inliers]
    center = ground.mean(axis=0)
    try:
        _, singular_values, vh = np.linalg.svd(ground - center, full_matrices=False)
    except np.linalg.LinAlgError:
        return result(_reject("ground_plane_not_found", count))
    if singular_values[1] <= 1e-10 * singular_values[0]:
        return result(_reject("ground_plane_not_found", count))
    ground_mask[np.flatnonzero(candidate)[inliers]] = True
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    slope = float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0))))
    residuals = np.abs((ground - center) @ normal)
    roughness = float(np.percentile(residuals, 95))
    if slope > limits.max_slope_deg:
        return result(LandingAssessment(False, "slope_too_high", count, slope, roughness))
    if roughness > limits.max_roughness_m:
        return result(LandingAssessment(False, "surface_too_rough", count, slope, roughness))

    heights = (cloud - center) @ normal
    raised = cloud[heights > limits.obstacle_height_m]
    clear_radius = -1.0
    if len(raised):
        clear_radius = float(np.min(np.linalg.norm(raised[:, :2], axis=1)))
        if clear_radius < limits.clearance_m:
            return result(LandingAssessment(
                False, "obstacle_inside_clearance", count, slope, roughness, clear_radius
            ))
    return result(LandingAssessment(True, "safe", count, slope, roughness, clear_radius))
