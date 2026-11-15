import time

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs.msg")
pytest.importorskip("interfaces.srv")

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from navigation.mission_node import MissionNode
from navigation.state import MissionState


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "ROS condition did not arrive before timeout"


def drain(executor, duration=0.2):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def test_real_ros_topics_stop_after_position_takeover():
    rclpy.init()
    mission = MissionNode()
    observer = Node("pilot_takeover_test_observer")
    executor = SingleThreadedExecutor()
    try:
        mission.timer.cancel()
        now = [10.0]
        mission._now = lambda: now[0]
        mission.position = (0.0, 0.0, -5.0)
        mission.destination = mission.original_destination = (8.0, 0.0, -5.0)
        mission.stamps = {key: now[0] for key in mission.stamps}
        qos = QoSProfile(
            depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        received = {"offboard": [], "setpoint": [], "command": []}
        subscriptions = [
            observer.create_subscription(msg_type, topic, received[key].append, qos)
            for key, msg_type, topic in (
                ("offboard", OffboardControlMode, "/fmu/in/offboard_control_mode"),
                ("setpoint", TrajectorySetpoint, "/fmu/in/trajectory_setpoint"),
                ("command", VehicleCommand, "/fmu/in/vehicle_command"),
            )
        ]
        status_pub = observer.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", qos)
        client = observer.create_client(SetBool, "/autonomy/enable")
        executor.add_node(mission)
        executor.add_node(observer)
        spin_until(executor, lambda: (
            status_pub.get_subscription_count() > 0
            and all(pub.get_subscription_count() > 0 for pub in (
                mission.offboard_pub, mission.setpoint_pub, mission.command_pub,
            ))
            and client.service_is_ready()
        ))
        future = client.call_async(SetBool.Request(data=True))
        spin_until(executor, future.done)
        assert future.result().success
        for _ in range(10):
            mission._tick()
            drain(executor, 0.02)
        spin_until(executor, lambda: all(received.values()))
        assert mission.state == MissionState.TAKEOFF

        status_pub.publish(VehicleStatus(
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            arming_state=VehicleStatus.ARMING_STATE_ARMED,
        ))
        spin_until(executor, lambda: mission.offboard_seen)
        status_pub.publish(VehicleStatus(
            nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL,
            arming_state=VehicleStatus.ARMING_STATE_ARMED,
        ))
        spin_until(executor, lambda: mission.state == MissionState.PILOT_OVERRIDE)
        drain(executor)
        baseline = {key: len(messages) for key, messages in received.items()}

        for elapsed in (0.1, 9.0, 301.0, 600.0):
            now[0] = 10.0 + elapsed
            mission._tick()
            drain(executor, 0.02)
        status_pub.publish(VehicleStatus(
            nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            arming_state=VehicleStatus.ARMING_STATE_ARMED,
        ))
        drain(executor)
        future = client.call_async(SetBool.Request(data=True))
        spin_until(executor, future.done)
        assert not future.result().success
        mission._tick()
        drain(executor)
        assert mission.state == MissionState.PILOT_OVERRIDE
        assert {key: len(messages) for key, messages in received.items()} == baseline
        assert all(
            msg.command != VehicleCommand.VEHICLE_CMD_NAV_LAND
            for msg in received["command"]
        )
        assert len(subscriptions) == 3  # Retain subscriptions through the assertion.
    finally:
        executor.shutdown()
        observer.destroy_node()
        mission.destroy_node()
        rclpy.shutdown()
