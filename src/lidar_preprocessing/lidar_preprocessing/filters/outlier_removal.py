# Imports
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from scipy.spatial import cKDTree

def statistical_outlier_removal(point_cloud : PointCloud2, meanK=30, threshold=2) -> PointCloud2:
    """
    Removes points that are far from their neighboring points using Statistical Outlier Removal (SOR).

    Time Complexity: O(nlogn)

    Args:
        point_cloud (PointCloud2): Input point cloud to filter.
        meanK (int): Number of nearest neighbors used to compute the averagedistance for each point.
        threshold (float): Standard deviation multiplier used to decide outliers.
            - Smaller value: removes more points.
            - Larger value: removes fewer points.

    Returns:
        PointCloud2: Filtered point cloud with statistical outliers removed.

    """

    field_names = ("x", "y", "z")

    # Reads all (x, y, z) points from the PointCloud2 message as a Numpy array of shape (n, 3) 
    points = point_cloud2.read_points_numpy(point_cloud, field_names, skip_nans = True).astype(np.float32)

    # Keep only points where x, y, z are all finite
    points = points[np.all(np.isfinite(points), axis=1)]

    if len(points) == 0:
        return point_cloud

    if len(points) <= meanK:
        return point_cloud
    
    # Build KDTree for fast nearest-neighbor search
    tree = cKDTree(points)
    
    # Query meanK + 1 because the nearest neighbor is the point itself
    distances, _ = tree.query(points, k=meanK + 1)

    # Remove distance to itself, which is column 0
    neighbor_distances = distances[:, 1:]

    average_distances = np.mean(neighbor_distances, axis=1)

    global_mean = np.mean(average_distances)
    global_std = np.std(average_distances)

    distance_limit = global_mean + threshold * global_std

    mask = average_distances <= distance_limit
    filtered_points = points[mask]

    return point_cloud2.create_cloud_xyz32(
        point_cloud.header,
        filtered_points.tolist()
    )