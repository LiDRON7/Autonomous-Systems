"""ROS node for immediate obstacle distance and landing assessment."""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String

from .depth import minimum_depth
from .landing import LandingLimits, assess_landing_zone


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception")
        self.declare_parameter("depth_topic", "/oakd/depth/image")
        self.declare_parameter("lidar_topic", "/lidar/points")
        self.declare_parameter("obstacle_threshold_m", 1.5)
        self.declare_parameter("depth_roi_ratio", 0.4)
        self.declare_parameter("landing.footprint_m", 1.2)
        self.declare_parameter("landing.min_points", 80)
        self.declare_parameter("landing.max_slope_deg", 8.0)
        self.declare_parameter("landing.max_roughness_m", 0.08)
        self.declare_parameter("landing.obstacle_height_m", 0.20)
        self.declare_parameter("landing.clearance_m", 0.75)

        self.distance_pub = self.create_publisher(Float32, "/perception/obstacle_distance", 10)
        self.detected_pub = self.create_publisher(Bool, "/perception/obstacle_detected", 10)
        self.landing_pub = self.create_publisher(String, "/landing/assessment", 10)
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._on_depth, 10
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("lidar_topic").value), self._on_lidar, 10
        )

    def _on_depth(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        if encoding in {"32fc1", "32fc"}:
            image = np.frombuffer(msg.data, dtype=np.float32)
        elif encoding in {"16uc1", "mono16"}:
            image = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) / 1000.0
        else:
            self.get_logger().warning(f"Unsupported depth encoding: {msg.encoding}")
            return
        expected = msg.height * msg.width
        if image.size < expected:
            self.get_logger().warning("Truncated depth image")
            return
        result = minimum_depth(
            image[:expected].reshape((msg.height, msg.width)),
            float(self.get_parameter("obstacle_threshold_m").value),
            float(self.get_parameter("depth_roi_ratio").value),
        )
        distance = Float32(data=result.distance_m)
        detected = Bool(data=result.detected)
        self.distance_pub.publish(distance)
        self.detected_pub.publish(detected)

    def _on_lidar(self, msg: PointCloud2) -> None:
        points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        limits = LandingLimits(
            footprint_m=float(self.get_parameter("landing.footprint_m").value),
            min_points=int(self.get_parameter("landing.min_points").value),
            max_slope_deg=float(self.get_parameter("landing.max_slope_deg").value),
            max_roughness_m=float(self.get_parameter("landing.max_roughness_m").value),
            obstacle_height_m=float(self.get_parameter("landing.obstacle_height_m").value),
            clearance_m=float(self.get_parameter("landing.clearance_m").value),
        )
        assessment = assess_landing_zone(points, limits)
        payload = assessment.as_dict() | {
            "stamp_ns": self.get_clock().now().nanoseconds,
            "frame_id": msg.header.frame_id,
        }
        self.landing_pub.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
