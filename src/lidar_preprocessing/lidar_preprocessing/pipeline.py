from sensor_msgs.msg import PointCloud2
from .filters.nan_infinite_filter import nan_infinite_filter
from .filters.passthrough import passthrough_filter
from .filters.voxel_grid_downsampling import voxel_grid_downsampling
from .filters.outlier_removal import statistical_outlier_removal
from .filters.ground_segmentation import ransac_ground_segmentation


def _count_points(point_cloud: PointCloud2) -> int:
    return int(point_cloud.width * point_cloud.height)


def run_preprocessing_pipeline_with_stats(
    msg: PointCloud2,
    roi: dict[str, float] | None = None,
    ransac_params: dict[str, float] | None = None,
    voxel_size: float = 0.03,
    enable_nan_filter: bool = True,
    enable_voxel_filter: bool = True,
    enable_roi_filter: bool = True,
    enable_outlier_filter: bool = True,
    enable_ransac: bool = True,
    outlier_mean_k: int = 30,
    outlier_threshold: float = 2.0,
) -> tuple[PointCloud2, PointCloud2, PointCloud2, dict[str, int]]:
    """
    Primary pipeline function. 
    Runs all filters and returns: (filtered_msg, ground_msg, obstacles_msg, stage_counts)
    """

    # 1. Initialize data and stats
    filtered_msg = msg
    stage_counts: dict[str, int] = {
        "raw_input": _count_points(filtered_msg),
    }

    # 2. Sequential Filtering with Stats Tracking
    if enable_nan_filter:
        filtered_msg = nan_infinite_filter(filtered_msg)
    stage_counts["after_nan_removal"] = _count_points(filtered_msg)

    if enable_voxel_filter:
        filtered_msg = voxel_grid_downsampling(filtered_msg, voxel_size)
    stage_counts["after_voxel_grid_downsampling"] = _count_points(filtered_msg)

    if enable_roi_filter and roi is not None:
        filtered_msg = passthrough_filter(
            filtered_msg,
            x_min=roi["x_min"], x_max=roi["x_max"],
            y_min=roi["y_min"], y_max=roi["y_max"],
            z_min=roi["z_min"], z_max=roi["z_max"],
        )
    stage_counts["after_roi_filtering"] = _count_points(filtered_msg)

    if enable_outlier_filter:
        filtered_msg = statistical_outlier_removal(
            filtered_msg,
            meanK=outlier_mean_k,
            threshold=outlier_threshold,
        )
    stage_counts["after_statistical_outlier"] = _count_points(filtered_msg)

    # 3. Ground Segmentation (RANSAC)
    if enable_ransac:
        if ransac_params is None:
            ransac_params = {'dist_threshold': 0.08, 'num_iterations': 100}

        ground_msg, obstacles_msg = ransac_ground_segmentation(
            filtered_msg,
            dist_threshold=ransac_params["dist_threshold"],
            num_iterations=ransac_params["num_iterations"],
        )
    else:
        ground_msg = filtered_msg
        obstacles_msg = filtered_msg

    return filtered_msg, ground_msg, obstacles_msg, stage_counts


def run_preprocessing_pipeline(
    msg: PointCloud2,
    roi: dict[str, float] | None = None,
    ransac_params: dict[str, float] | None = None,
    voxel_size: float = 0.03,
    enable_nan_filter: bool = True,
    enable_voxel_filter: bool = True,
    enable_roi_filter: bool = True,
    enable_outlier_filter: bool = True,
    enable_ransac: bool = True, 
    outlier_mean_k: int = 30,
    outlier_threshold: float = 2.0,
) -> tuple[PointCloud2, PointCloud2, PointCloud2]:
    """
    Simplified API that returns only the three point clouds without the stats dictionary.
    """
    filtered_msg, ground_msg, obstacles_msg, _ = run_preprocessing_pipeline_with_stats(
        msg,
        roi=roi,
        ransac_params=ransac_params,
        voxel_size=voxel_size,
        enable_nan_filter=enable_nan_filter,
        enable_voxel_filter=enable_voxel_filter,
        enable_roi_filter=enable_roi_filter,
        enable_outlier_filter=enable_outlier_filter,
        enable_ransac=enable_ransac,
        outlier_mean_k=outlier_mean_k,
        outlier_threshold=outlier_threshold,
    )

    return filtered_msg, ground_msg, obstacles_msg