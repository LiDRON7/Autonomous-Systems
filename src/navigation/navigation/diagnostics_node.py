"""Read-only topic and timestamp diagnostics for simulation and hardware."""

from dataclasses import dataclass

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix, PointCloud2
from std_srvs.srv import Trigger


@dataclass
class TopicHealth:
    received_s: float | None = None
    frame_id: str = ""


class DiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics")
        self.declare_parameter("diagnostic_timeout_s", 2.0)
        self.health = {
            "oakd_depth_image": TopicHealth(),
            "oakd_depth_points": TopicHealth(),
            "lidar_points": TopicHealth(),
            "gps": TopicHealth(),
            "px4_position": TopicHealth(),
            "px4_status": TopicHealth(),
        }
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, "/oakd/depth/image", self._callback("oakd_depth_image"), 10)
        self.create_subscription(
            PointCloud2, "/oakd/depth/points", self._callback("oakd_depth_points"), 10
        )
        self.create_subscription(PointCloud2, "/lidar/points", self._callback("lidar_points"), 10)
        self.create_subscription(NavSatFix, "/gps", self._callback("gps"), 10)
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._callback("px4_position"),
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._callback("px4_status"), px4_qos
        )
        self.create_service(Trigger, "/autonomy/preflight_check", self._check)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _callback(self, name):
        def receive(msg):
            frame_id = getattr(getattr(msg, "header", None), "frame_id", "")
            self.health[name] = TopicHealth(self._now(), frame_id)
        return receive

    def _check(self, _request, response):
        timeout = float(self.get_parameter("diagnostic_timeout_s").value)
        now = self._now()
        stale = [
            name
            for name, health in self.health.items()
            if health.received_s is None or now - health.received_s > timeout
        ]
        missing_frames = [
            name
            for name in ("oakd_depth_image", "oakd_depth_points", "lidar_points", "gps")
            if self.health[name].received_s is not None and not self.health[name].frame_id
        ]
        response.success = not stale and not missing_frames
        details = []
        if stale:
            details.append("missing/stale: " + ", ".join(stale))
        if missing_frames:
            details.append("empty frame_id: " + ", ".join(missing_frames))
        response.message = "; ".join(details) if details else "all required inputs are healthy"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
