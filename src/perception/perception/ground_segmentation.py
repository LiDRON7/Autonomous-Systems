"""RANSAC plane segmentation independent of ROS."""

import numpy as np


def ransac_ground_inliers(
    points: np.ndarray,
    dist_threshold: float = 0.2,
    num_iterations: int = 100,
    min_points: int = 3,
) -> np.ndarray | None:
    """Return the dominant plane's inlier mask, or None without a valid model.

    Three non-collinear points define each candidate. A fixed random seed makes
    repeated assessments of the same cloud reproducible. Slope validation and
    SVD refinement belong to the landing evaluator, after segmentation.
    """
    cloud = np.asarray(points, dtype=float).reshape((-1, 3))
    required = max(3, min_points)
    if (
        len(cloud) < required
        or not np.isfinite(cloud).all()
        or not np.isfinite(dist_threshold)
        or dist_threshold <= 0.0
        or num_iterations < 1
    ):
        return None

    rng = np.random.default_rng(0)
    best_mask = None
    best_count = required - 1
    best_error = float("inf")
    for _ in range(num_iterations):
        a, b, c = cloud[rng.choice(len(cloud), 3, replace=False)]
        first, second = b - a, c - a
        normal = np.cross(first, second)
        norm = float(np.linalg.norm(normal))
        scale = float(np.linalg.norm(first) * np.linalg.norm(second))
        if scale == 0.0 or norm <= 1e-10 * scale:
            continue
        normal /= norm
        distances = np.abs((cloud - a) @ normal)
        mask = distances <= dist_threshold
        count = int(mask.sum())
        if count < required:
            continue
        error = float(distances[mask].mean())
        if count > best_count or (count == best_count and error < best_error):
            best_mask, best_count, best_error = mask, count, error
    return best_mask
