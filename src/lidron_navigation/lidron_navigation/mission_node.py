"""Unified PX4 mission controller for routing, avoidance, and landing."""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from lidron_interfaces.srv import SetGpsDestination, SetLocalDestination
from nav_msgs.msg import OccupancyGrid, Path
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .geo import gps_to_ned
from .grid import RollingGrid
from .planner import astar, simplify
from .state import MissionState, can_transition, data_is_fresh


ACTIVE_STATES = {
    MissionState.PREFLIGHT,
    MissionState.TAKEOFF,
    MissionState.PLANNING,
    MissionState.NAVIGATING,
    MissionState.REPLANNING,
    MissionState.APPROACH,
    MissionState.LANDING_CHECK,
    MissionState.HOLD,
}


class MissionNode(Node):
    """Own the single PX4 offboard-control stream for an autonomous mission."""

    def __init__(self) -> None:
        super().__init__("lidron_mission")
        self._declare_parameters()
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", px4_qos
        )
        self.state_pub = self.create_publisher(String, "/autonomy/state", 10)
        self.path_pub = self.create_publisher(Path, "/autonomy/path", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/autonomy/occupancy_grid", 10)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_position, px4_qos
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, px4_qos
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_attitude, px4_qos
        )
        self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected", self._on_land, px4_qos
        )
        self.create_subscription(
            VehicleCommandAck, "/fmu/out/vehicle_command_ack", self._on_command_ack, px4_qos
        )
        self.create_subscription(
            PointCloud2, "/oakd/depth/points", self._on_depth_points, 10
        )
        self.create_subscription(
            Bool, "/perception/obstacle_detected", self._on_obstacle, 10
        )
        self.create_subscription(
            String, "/landing/assessment", self._on_landing_assessment, 10
        )
        self.create_service(
            SetLocalDestination, "/autonomy/set_local_destination", self._set_local
        )
        self.create_service(
            SetGpsDestination, "/autonomy/set_gps_destination", self._set_gps
        )
        self.create_service(SetBool, "/autonomy/enable", self._enable)
        self.create_service(Trigger, "/autonomy/abort", self._abort)

        self.state = MissionState.IDLE
        self.position: tuple[float, float, float] | None = None
        self.reference: tuple[float, float, float] | None = None
        self.destination: tuple[float, float, float] | None = None
        self.original_destination: tuple[float, float, float] | None = None
        self.yaw = 0.0
        self.vehicle_status: VehicleStatus | None = None
        self.landed = True
        self.stamps: dict[str, float | None] = {
            "position": None,
            "depth": None,
            "lidar": None,
            "status": None,
        }
        self.obstacle_detected = False
        self.landing_suitable = False
        self.landing_confirmations = 0
        self.grid = RollingGrid(
            resolution=float(self.get_parameter("map.resolution").value),
            size_m=float(self.get_parameter("map.size_m").value),
            inflation_m=float(self.get_parameter("map.inflation_m").value),
            expiry_s=float(self.get_parameter("map.expiry_s").value),
        )
        self.grid_origin = (0.0, 0.0)
        self.path: list[tuple[float, float]] = []
        self.path_index = 0
        self.hold_point: tuple[float, float, float] | None = None
        self.state_started = self._now()
        self.mission_started: float | None = None
        self.offboard_warmup = 0
        self.last_command_ack: tuple[int, int] | None = None
        self.search_index = 0
        self.timer = self.create_timer(0.1, self._tick)
        if bool(self.get_parameter("auto_start").value):
            self.destination = (
                float(self.get_parameter("destination.north").value),
                float(self.get_parameter("destination.east").value),
                float(self.get_parameter("destination.down").value),
            )
            self.original_destination = self.destination
            self.mission_started = self._now()
            self._transition(MissionState.PREFLIGHT)

    def _declare_parameters(self) -> None:
        values = {
            "auto_start": False,
            "cruise_altitude_m": 5.0,
            "arrival_tolerance_m": 0.75,
            "sensor_timeout_s": 1.0,
            "hold_timeout_s": 8.0,
            "mission_timeout_s": 300.0,
            "landing_confirmations": 8,
            "landing_check_timeout_s": 5.0,
            "landing_search_step_m": 1.5,
            "landing_search_limit": 8,
            "map.resolution": 0.5,
            "map.size_m": 30.0,
            "map.inflation_m": 1.0,
            "map.expiry_s": 2.0,
            "map.min_height_m": -1.5,
            "map.max_height_m": 1.5,
            "destination.north": 8.0,
            "destination.east": 0.0,
            "destination.down": -5.0,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _transition(self, state: MissionState) -> None:
        if state == self.state:
            return
        if not can_transition(self.state, state):
            raise RuntimeError(f"invalid mission transition: {self.state.value} -> {state.value}")
        self.get_logger().info(f"Mission {self.state.value} -> {state.value}")
        self.state = state
        self.state_started = self._now()
        if state == MissionState.LANDING_CHECK:
            self.landing_confirmations = 0
        if state == MissionState.HOLD and self.position:
            self.hold_point = self.position

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        if not (getattr(msg, "xy_valid", True) and getattr(msg, "z_valid", True)):
            return
        self.position = float(msg.x), float(msg.y), float(msg.z)
        if getattr(msg, "xy_global", False) and getattr(msg, "z_global", False):
            self.reference = float(msg.ref_lat), float(msg.ref_lon), float(msg.ref_alt)
        self.stamps["position"] = self._now()

    def _on_status(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self.stamps["status"] = self._now()

    def _on_attitude(self, msg: VehicleAttitude) -> None:
        w, x, y, z = (float(value) for value in msg.q)
        self.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _on_land(self, msg: VehicleLandDetected) -> None:
        self.landed = bool(msg.landed)

    def _on_command_ack(self, msg: VehicleCommandAck) -> None:
        self.last_command_ack = int(msg.command), int(msg.result)
        accepted = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        }
        if int(msg.result) not in accepted and self.state in ACTIVE_STATES:
            self._safe_abort(f"PX4 rejected command {msg.command}: result {msg.result}")

    def _on_obstacle(self, msg: Bool) -> None:
        self.obstacle_detected = bool(msg.data)

    def _on_landing_assessment(self, msg: String) -> None:
        try:
            assessment = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning("Ignored malformed landing assessment")
            return
        self.landing_suitable = bool(assessment.get("suitable", False))
        self.stamps["lidar"] = self._now()
        if self.state == MissionState.LANDING_CHECK:
            self.landing_confirmations = (
                self.landing_confirmations + 1 if self.landing_suitable else 0
            )

    def _on_depth_points(self, msg: PointCloud2) -> None:
        if self.position is None:
            return
        raw = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        points = np.asarray(raw, dtype=float).reshape((-1, 3))
        points = points[np.isfinite(points).all(axis=1)]
        low = float(self.get_parameter("map.min_height_m").value)
        high = float(self.get_parameter("map.max_height_m").value)
        points = points[(points[:, 1] >= low) & (points[:, 1] <= high)]
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        world = [
            (
                self.position[0] + cos_yaw * forward - sin_yaw * right,
                self.position[1] + sin_yaw * forward + cos_yaw * right,
            )
            for right, _vertical, forward in points
            if forward > 0.2
        ]
        self.grid_origin = self.grid.origin_around(self.position[:2])
        self.grid.observe(world, self.grid_origin, self._now())
        self.stamps["depth"] = self._now()
        self._publish_grid(msg.header.frame_id or "map")

    def _set_local(self, request, response):
        if self.state not in {MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT}:
            response.accepted = False
            response.message = "mission is active"
            return response
        self.destination = request.north, request.east, request.down
        self.original_destination = self.destination
        response.accepted = True
        response.message = "local NED destination stored"
        return response

    def _set_gps(self, request, response):
        if self.reference is None:
            response.accepted = False
            response.message = "PX4 global home reference is unavailable"
            return response
        local = gps_to_ned(
            request.latitude, request.longitude, request.altitude, self.reference
        )
        local_request = type("Local", (), {
            "north": local[0], "east": local[1], "down": local[2]
        })()
        return self._set_local(local_request, response)

    def _enable(self, request, response):
        if not request.data:
            if self.state not in ACTIVE_STATES:
                response.success, response.message = False, "mission is not active"
                return response
            self._transition(MissionState.HOLD)
            response.success, response.message = True, "mission holding"
            return response
        if self.destination is None:
            response.success, response.message = False, "set a destination first"
            return response
        if self.state not in {MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT}:
            response.success, response.message = False, "mission is already active"
            return response
        self.mission_started = self._now()
        self.offboard_warmup = 0
        self.search_index = 0
        self._transition(MissionState.PREFLIGHT)
        response.success, response.message = True, "preflight started"
        return response

    def _abort(self, _request, response):
        if self.state in {MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT}:
            response.success, response.message = False, "no active mission"
            return response
        self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._transition(MissionState.ABORT)
        response.success, response.message = True, "controlled landing requested"
        return response

    def _health_ok(self) -> bool:
        now = self._now()
        timeout = float(self.get_parameter("sensor_timeout_s").value)
        return all(data_is_fresh(now, self.stamps[key], timeout) for key in self.stamps)

    def _tick(self) -> None:
        self.state_pub.publish(String(data=self.state.value))
        if self.state in ACTIVE_STATES:
            self._publish_offboard_mode()
        if self.mission_started and self._now() - self.mission_started > float(
            self.get_parameter("mission_timeout_s").value
        ):
            self._safe_abort("mission timeout")
            return
        if self.state == MissionState.IDLE:
            return
        if self.state in ACTIVE_STATES and not self._health_ok():
            if self.state != MissionState.PREFLIGHT:
                self._transition(MissionState.HOLD)
            self._hold()
            if self._now() - self.state_started > float(self.get_parameter("hold_timeout_s").value):
                self._safe_abort("required data timed out")
            return
        handlers = {
            MissionState.PREFLIGHT: self._preflight,
            MissionState.TAKEOFF: self._takeoff,
            MissionState.PLANNING: self._plan,
            MissionState.REPLANNING: self._plan,
            MissionState.NAVIGATING: self._navigate,
            MissionState.APPROACH: self._approach,
            MissionState.LANDING_CHECK: self._landing_check,
            MissionState.HOLD: self._recover_from_hold,
            MissionState.LANDING: self._wait_for_landing,
        }
        handler = handlers.get(self.state)
        if handler:
            handler()

    def _preflight(self) -> None:
        if self.position is None or not self._health_ok():
            return
        cruise_z = -abs(float(self.get_parameter("cruise_altitude_m").value))
        self._setpoint(self.position[0], self.position[1], cruise_z, self.yaw)
        self.offboard_warmup += 1
        if self.offboard_warmup >= 10:
            self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._transition(MissionState.TAKEOFF)

    def _takeoff(self) -> None:
        assert self.position is not None
        cruise_z = -abs(float(self.get_parameter("cruise_altitude_m").value))
        self._setpoint(self.position[0], self.position[1], cruise_z, self.yaw)
        if abs(self.position[2] - cruise_z) <= 0.5:
            self._transition(MissionState.PLANNING)

    def _planning_goal(self) -> tuple[float, float]:
        assert self.position and self.destination
        half = self.grid.width * self.grid.resolution / 2.0 - self.grid.resolution
        dx = self.destination[0] - self.position[0]
        dy = self.destination[1] - self.position[1]
        scale = min(1.0, half / max(abs(dx), abs(dy), half))
        return self.position[0] + dx * scale, self.position[1] + dy * scale

    def _plan(self) -> None:
        assert self.position and self.destination
        self.grid_origin = self.grid.origin_around(self.position[:2])
        start = self.grid.to_cell(self.position[:2], self.grid_origin)
        goal = self.grid.to_cell(self._planning_goal(), self.grid_origin)
        route = astar(start, goal, set(self.grid.occupied), self.grid.width)
        route = simplify(route, set(self.grid.occupied))
        if not route:
            self._transition(MissionState.HOLD)
            return
        self.path = [self.grid.to_world(cell, self.grid_origin) for cell in route]
        self.path_index = 1 if len(self.path) > 1 else 0
        self._publish_path()
        self._transition(MissionState.NAVIGATING)

    def _navigate(self) -> None:
        assert self.position and self.destination
        if self.obstacle_detected:
            self.hold_point = self.position
            self._hold()
            self._transition(MissionState.REPLANNING)
            return
        if self.path_index >= len(self.path):
            if math.dist(self.position[:2], self.destination[:2]) <= float(
                self.get_parameter("arrival_tolerance_m").value
            ):
                self._transition(MissionState.APPROACH)
            else:
                self._transition(MissionState.PLANNING)
            return
        north, east = self.path[self.path_index]
        target_z = self.destination[2]
        yaw = math.atan2(east - self.position[1], north - self.position[0])
        self._setpoint(north, east, target_z, yaw)
        if math.dist(self.position[:2], (north, east)) <= float(
            self.get_parameter("arrival_tolerance_m").value
        ):
            self.path_index += 1

    def _approach(self) -> None:
        assert self.position and self.destination
        self._setpoint(*self.destination, self.yaw)
        if math.dist(self.position, self.destination) <= float(
            self.get_parameter("arrival_tolerance_m").value
        ):
            self._transition(MissionState.LANDING_CHECK)

    def _landing_check(self) -> None:
        assert self.position
        self._setpoint(*self.position, self.yaw)
        required = int(self.get_parameter("landing_confirmations").value)
        if self.landing_confirmations >= required:
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._transition(MissionState.LANDING)
            return
        if self._now() - self.state_started <= float(
            self.get_parameter("landing_check_timeout_s").value
        ):
            return
        if not self._next_search_destination():
            self._safe_abort("no safe landing zone found")

    def _next_search_destination(self) -> bool:
        assert self.original_destination
        limit = int(self.get_parameter("landing_search_limit").value)
        if self.search_index >= limit:
            return False
        offsets = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))
        dx, dy = offsets[self.search_index % len(offsets)]
        ring = 1 + self.search_index // len(offsets)
        step = float(self.get_parameter("landing_search_step_m").value) * ring
        self.search_index += 1
        self.destination = (
            self.original_destination[0] + dx * step,
            self.original_destination[1] + dy * step,
            self.original_destination[2],
        )
        self._transition(MissionState.PLANNING)
        return True

    def _recover_from_hold(self) -> None:
        self._hold()
        if self._health_ok() and not self.obstacle_detected:
            self._transition(MissionState.REPLANNING)

    def _wait_for_landing(self) -> None:
        if self.landed:
            self._transition(MissionState.COMPLETE)

    def _hold(self) -> None:
        target = self.hold_point or self.position
        if target:
            self._setpoint(*target, self.yaw)

    def _safe_abort(self, reason: str) -> None:
        if self.state in {MissionState.ABORT, MissionState.COMPLETE}:
            return
        self.get_logger().error(reason)
        self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._transition(MissionState.ABORT)

    def _publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _setpoint(self, north: float, east: float, down: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(north), float(east), float(down)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _command(self, command: int, **parameters: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        for index in range(1, 8):
            setattr(msg, f"param{index}", float(parameters.get(f"param{index}", 0.0)))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def _publish_path(self) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for north, east in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = north
            pose.pose.position.y = east
            pose.pose.position.z = self.destination[2] if self.destination else 0.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def _publish_grid(self, _sensor_frame: str) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width = msg.info.height = self.grid.width
        msg.info.origin.position.x = self.grid_origin[0]
        msg.info.origin.position.y = self.grid_origin[1]
        msg.data = [
            100 if (x, y) in self.grid.occupied else 0
            for y in range(self.grid.width)
            for x in range(self.grid.width)
        ]
        self.grid_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
