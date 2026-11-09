"""Read-only topic and timestamp diagnostics for simulation and hardware."""

from dataclasses import dataclass

import rclpy
from px4_msgs.msg import BatteryStatus, SensorGps, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Trigger


@dataclass
class TopicHealth:
    received_s: float | None = None
    frame_id: str = ""


class DiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics")
        self.declare_parameter("diagnostic_timeout_s", 2.0)
        self.declare_parameter("require_lidar", True)
        self.vehicle_status: VehicleStatus | None = None
        self.battery_status: BatteryStatus | None = None
        self.health = {
            "oakd_depth_image": TopicHealth(),
            "oakd_depth_points": TopicHealth(),
            "lidar_points": TopicHealth(),
            "gps": TopicHealth(),
            "px4_position": TopicHealth(),
            "px4_status": TopicHealth(),
            "battery": TopicHealth(),
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
        # PX4 publishes GPS on hardware and in SITL, so no separate /gps driver is needed.
        self.create_subscription(
            SensorGps, "/fmu/out/vehicle_gps_position", self._callback("gps"), px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._callback("px4_position"),
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, px4_qos
        )
        self.create_subscription(
            BatteryStatus, "/fmu/out/battery_status", self._on_battery, px4_qos
        )
        self.create_service(Trigger, "/autonomy/preflight_check", self._check)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _callback(self, name):
        def receive(msg):
            frame_id = getattr(getattr(msg, "header", None), "frame_id", "")
            self.health[name] = TopicHealth(self._now(), frame_id)
        return receive

    def _on_status(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self.health["px4_status"] = TopicHealth(self._now())

    def _on_battery(self, msg: BatteryStatus) -> None:
        self.battery_status = msg
        self.health["battery"] = TopicHealth(self._now())

    def _check(self, _request, response):
        timeout = float(self.get_parameter("diagnostic_timeout_s").value)
        require_lidar = bool(self.get_parameter("require_lidar").value)
        now = self._now()
        stale = [
            name
            for name, health in self.health.items()
            if (name != "lidar_points" or require_lidar)
            and (health.received_s is None or now - health.received_s > timeout)
        ]
        missing_frames = [
            name
            for name in ("oakd_depth_image", "oakd_depth_points", "lidar_points")
            if self.health[name].received_s is not None and not self.health[name].frame_id
        ]
        details = []
        if stale:
            details.append("missing/stale: " + ", ".join(stale))
        if missing_frames:
            details.append("empty frame_id: " + ", ".join(missing_frames))
        # Missing/stale status already fails above; only interpret fresh telemetry.
        if "px4_status" not in stale and self.vehicle_status is not None:
            if not self.vehicle_status.pre_flight_checks_pass:
                details.append("PX4 preflight checks have not passed")
            if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
                details.append("drone is not disarmed")
        if "battery" not in stale and self.battery_status is not None:
            if not self.battery_status.connected:
                details.append("battery is not connected")
            if self.battery_status.warning != BatteryStatus.BATTERY_WARNING_NONE:
                details.append(f"battery warning active: {self.battery_status.warning}")
        response.success = not details
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
