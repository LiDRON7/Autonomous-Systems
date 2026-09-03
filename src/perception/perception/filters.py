"""Point-cloud filters used before landing-zone evaluation."""

import numpy as np
from scipy.spatial import cKDTree


def finite_points(points: np.ndarray) -> np.ndarray:
    """Return only finite XYZ points."""
    cloud = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    return cloud[np.isfinite(cloud).all(axis=1)]


def voxel_downsample(points: np.ndarray, voxel_size_m: float = 0.03) -> np.ndarray:
    """Keep one representative point from each occupied voxel."""
    cloud = finite_points(points)
    if not len(cloud) or voxel_size_m <= 0.0:
        return cloud
    cells = np.floor(cloud / voxel_size_m).astype(np.int64)
    _, first_indices = np.unique(cells, axis=0, return_index=True)
    return cloud[np.sort(first_indices)]


def statistical_outlier_removal(
    points: np.ndarray,
    mean_k: int = 50,
    threshold: float = 3.0,
) -> np.ndarray:
    """Remove sparse outliers with a KD-tree in O(n log n) time."""
    cloud = finite_points(points)
    if len(cloud) <= mean_k or mean_k < 1:
        return cloud
    distances, _ = cKDTree(cloud).query(cloud, k=mean_k + 1)
    average_distances = distances[:, 1:].mean(axis=1)
    distance_limit = average_distances.mean() + threshold * average_distances.std()
    return cloud[average_distances <= distance_limit]


def preprocess_landing_cloud(
    points: np.ndarray,
    voxel_size_m: float = 0.03,
    mean_k: int = 50,
    threshold: float = 3.0,
) -> np.ndarray:
    """Apply finite-value, voxel, and statistical-outlier filters."""
    downsampled = voxel_downsample(points, voxel_size_m)
    return statistical_outlier_removal(downsampled, mean_k, threshold)
