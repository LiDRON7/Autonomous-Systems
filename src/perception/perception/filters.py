"""Point-cloud filters used before landing-zone evaluation."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class PassThroughBounds:
    """Inclusive limits in the input cloud frame; Z is unbounded by default."""

    x_min: float = -10.0
    x_max: float = 10.0
    y_min: float = -10.0
    y_max: float = 10.0
    z_min: float = -float("inf")
    z_max: float = float("inf")

    def __post_init__(self) -> None:
        for axis in ("x", "y", "z"):
            low, high = getattr(self, f"{axis}_min"), getattr(self, f"{axis}_max")
            if np.isnan(low) or np.isnan(high) or low > high or low == np.inf or high == -np.inf:
                raise ValueError(f"invalid ROI bounds for {axis}")

    def validate_landing_area(self, footprint_m: float, clearance_m: float) -> None:
        """Prevent an XY crop from removing required landing/clearance data."""
        if not np.isfinite(footprint_m) or not np.isfinite(clearance_m):
            raise ValueError("landing footprint and clearance must be finite")
        if footprint_m <= 0.0 or clearance_m < 0.0:
            raise ValueError("landing footprint must be positive and clearance non-negative")
        extent = max(footprint_m / 2.0, clearance_m)
        if self.x_min > -extent or self.x_max < extent or self.y_min > -extent or self.y_max < extent:
            raise ValueError("ROI must contain the landing footprint and clearance area")


def finite_points(points: np.ndarray) -> np.ndarray:
    """Return only finite XYZ points."""
    cloud = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    return cloud[np.isfinite(cloud).all(axis=1)]


def passthrough_filter(
    points: np.ndarray, bounds: PassThroughBounds | None = None,
) -> np.ndarray:
    """Keep finite XYZ points inside the configured processing volume."""
    bounds = bounds or PassThroughBounds()
    cloud = finite_points(points)
    lower = np.array([bounds.x_min, bounds.y_min, bounds.z_min], dtype=cloud.dtype)
    upper = np.array([bounds.x_max, bounds.y_max, bounds.z_max], dtype=cloud.dtype)
    return cloud[((cloud >= lower) & (cloud <= upper)).all(axis=1)]


def exclude_box(
    points: np.ndarray, bounds: PassThroughBounds | None = None,
) -> np.ndarray:
    """Remove finite XYZ points inside an inclusive axis-aligned box."""
    cloud = finite_points(points)
    if bounds is None:
        return cloud
    lower = np.array([bounds.x_min, bounds.y_min, bounds.z_min], dtype=cloud.dtype)
    upper = np.array([bounds.x_max, bounds.y_max, bounds.z_max], dtype=cloud.dtype)
    inside = ((cloud >= lower) & (cloud <= upper)).all(axis=1)
    return cloud[~inside]


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
    roi: PassThroughBounds | None = None,
    exclusion: PassThroughBounds | None = None,
) -> np.ndarray:
    """Apply finite, ROI, self-exclusion, voxel, and outlier filters."""
    cropped = passthrough_filter(points, roi)
    self_filtered = exclude_box(cropped, exclusion)
    downsampled = voxel_downsample(self_filtered, voxel_size_m)
    return statistical_outlier_removal(downsampled, mean_k, threshold)
