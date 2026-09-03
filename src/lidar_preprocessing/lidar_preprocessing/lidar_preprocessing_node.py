# This imports the ROS2 Python Client Library. It lets python talk to ROS2.
import rclpy

# This imports the Node class.
from rclpy.node import Node

# This imports the PointCloud2 message type, which is used to represent 3D point cloud data.
from sensor_msgs.msg import PointCloud2

# We use run_preprocessing_pipeline_with_stats to get both the clouds and the logging data
from .pipeline import run_preprocessing_pipeline_with_stats


class LidarPreprocessingNode(Node):
    def __init__(self):
        # Initialize the node with the name 'lidar_preprocessing_node'
        super().__init__('lidar_preprocessing_node')

        # Topic Names
        self.input_topic = '/lidar/points/points'
        self.output_topic = '/lidar/points/filtered'
        self.ground_topic = '/lidar/points/ground'
        self.obstacles_topic = '/lidar/points/obstacles'

        # ROI Parameters (Wider range for better coverage)
        self.declare_parameter('roi.x_min', -10.0)
        self.declare_parameter('roi.x_max', 10.0)
        self.declare_parameter('roi.y_min', -10.0)
        self.declare_parameter('roi.y_max', 10.0)
        self.declare_parameter('roi.z_min', -2.0)
        self.declare_parameter('roi.z_max', 5.0)

        # Filter Toggles
        self.declare_parameter('filters.enable_nan', True)
        self.declare_parameter('filters.enable_voxel', True)
        self.declare_parameter('filters.enable_roi', True)
        self.declare_parameter('filters.enable_outlier', True)
        self.declare_parameter('filters.enable_ransac', True)

        # Algorithm Parameters
        self.declare_parameter('voxel.size', 0.03)
        self.declare_parameter('outlier.mean_k', 50)
        self.declare_parameter('outlier.threshold', 3.0)
        self.declare_parameter('ransac.dist_threshold', 0.2)
        self.declare_parameter('ransac.num_iterations', 100)

        # Subscriptions
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.ground_publisher = self.create_publisher(PointCloud2, self.ground_topic, 10)
        self.obstacles_publisher = self.create_publisher(PointCloud2, self.obstacles_topic, 10)

        # Logging initialization
        self.get_logger().info(f'Node started. Subscribed to {self.input_topic}')
        self.get_logger().info(f'Publishing ground to {self.ground_topic}')
        self.get_logger().info(f'Publishing obstacles to {self.obstacles_topic}')

    def pointcloud_callback(self, msg: PointCloud2):
        # 1. Extract All Parameters
        roi = {
            'x_min': float(self.get_parameter('roi.x_min').value),
            'x_max': float(self.get_parameter('roi.x_max').value),
            'y_min': float(self.get_parameter('roi.y_min').value),
            'y_max': float(self.get_parameter('roi.y_max').value),
            'z_min': float(self.get_parameter('roi.z_min').value),
            'z_max': float(self.get_parameter('roi.z_max').value),
        }

        ransac_params = {
            'dist_threshold': float(self.get_parameter('ransac.dist_threshold').value),
            'num_iterations': int(self.get_parameter('ransac.num_iterations').value),
        }

        # 2. Run the preprocessing pipeline
        # Unpacks 4 items: Filtered Cloud, Ground Cloud, Obstacles Cloud, and the Stats Dict
        filtered_msg, ground_msg, obstacles_msg, stage_counts = run_preprocessing_pipeline_with_stats(
            msg,
            roi=roi,
            ransac_params=ransac_params,
            voxel_size=float(self.get_parameter('voxel.size').value),
            enable_nan_filter=bool(self.get_parameter('filters.enable_nan').value),
            enable_voxel_filter=bool(self.get_parameter('filters.enable_voxel').value),
            enable_roi_filter=bool(self.get_parameter('filters.enable_roi').value),
            enable_outlier_filter=bool(self.get_parameter('filters.enable_outlier').value),
            enable_ransac=bool(self.get_parameter('filters.enable_ransac').value),
            outlier_mean_k=int(self.get_parameter('outlier.mean_k').value),
            outlier_threshold=float(self.get_parameter('outlier.threshold').value),
        )

        # 3. Calculate Statistics (based on the final filter stage in stats)
        num_points_before = stage_counts['raw_input']
        num_points_after = stage_counts['after_statistical_outlier']
        
        if num_points_before > 0:
            reduction = (1 - (num_points_after / num_points_before)) * 100
        else:
            reduction = 0.0

        # 4. Log summary
        self.get_logger().info(
            f'Points In: {num_points_before} | Points Out: {num_points_after} | '
            f'Reduction: {reduction:.2f}%'
        )

        # 5. Publish all three clouds
        self.publisher.publish(filtered_msg)
        self.ground_publisher.publish(ground_msg)
        self.obstacles_publisher.publish(obstacles_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()