"""ROS integration checks; run in the sourced autonomy container."""

import json
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

from perception.node import PerceptionNode


@pytest.mark.parametrize("scenario,reason", [
    ("flat", "safe"),
    ("raised", "obstacle_inside_clearance"),
    ("degenerate", "ground_plane_not_found"),
    ("cropped", "safe"),
])
def test_lidar_topics_publish_assessment_and_partition(scenario, reason):
    rclpy.init()
    perception = PerceptionNode()
    observer = rclpy.create_node("landing_test_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(perception)
    executor.add_node(observer)
    received = {}
    subscriptions = []
    for name, message_type in [("assessment", String), ("ground_points", PointCloud2),
                               ("non_ground_points", PointCloud2)]:
        subscriptions.append(observer.create_subscription(
            message_type, f"/landing/{name}",
            lambda msg, key=name: received.__setitem__(key, msg), 10,
        ))
    publisher = observer.create_publisher(PointCloud2, "/lidar/points", 10)
    try:
        # Keep the deliberately small raised cluster through preprocessing.
        results = perception.set_parameters([
            Parameter("filters.outlier_mean_k", value=1000),
            Parameter("ransac.dist_threshold", value=0.03),
            Parameter("ransac.num_iterations", value=150),
        ])
        assert all(result.successful for result in results)
        axis = np.linspace(-0.5, 0.5, 20)
        points = np.array([(x, y, 0.0) for x in axis for y in axis])
        if scenario == "raised":
            points = np.vstack([points, [[0.2, 0.2, 0.5], [0.3, 0.2, 0.5]]])
        elif scenario == "degenerate":
            points = np.array([(x, 0.0, 0.0) for x in np.linspace(-0.5, 0.5, 100)])
            # Retain enough collinear points to reach RANSAC.
            perception.set_parameters([Parameter("filters.voxel_size_m", value=0.0)])
        elif scenario == "cropped":
            points = np.vstack([points, [[11, 0, 0], [0, 11, 0], [0, 0, 11]]])
            assert perception.set_parameters_atomically([
                Parameter("roi.z_enabled", value=True),
                Parameter("roi.z_min", value=-1.0),
                Parameter("roi.z_max", value=1.0),
            ]).successful
        header = Header(frame_id="lidar_frame")
        header.stamp = observer.get_clock().now().to_msg()
        message = point_cloud2.create_cloud_xyz32(header, points.tolist())
        deadline = time.monotonic() + 5.0
        while len(received) < 3 and time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
        assert len(received) == 3
        assessment = json.loads(received["assessment"].data)
        assert assessment["reason"] == reason
        assert assessment["suitable"] == (reason == "safe")
        expected_count = len(points) - (3 if scenario == "cropped" else 0)
        assert assessment["input_points"] == len(points)
        assert assessment["filtered_points"] == expected_count
        assert assessment["ground_points"] + assessment["non_ground_points"] == expected_count
        assert assessment["frame_id"] == "lidar_frame"
        for name in ("ground_points", "non_ground_points"):
            cloud = received[name]
            assert cloud.header == header
            assert cloud.width * cloud.height == assessment[name]
        if scenario == "raised":
            assert assessment["non_ground_points"] == 2
            assert assessment["roughness_m"] < 0.001
    finally:
        executor.shutdown()
        observer.destroy_node()
        perception.destroy_node()
        rclpy.shutdown()


def test_roi_parameter_updates_preserve_required_landing_area():
    rclpy.init()
    node = PerceptionNode()
    try:
        for parameter in [
            Parameter("roi.x_min", value=-0.6),
            Parameter("roi.y_max", value=0.6),
            Parameter("roi.z_min", value=11.0),
            Parameter("landing.footprint_m", value=30.0),
            Parameter("landing.clearance_m", value=11.0),
        ]:
            result = node.set_parameters_atomically([parameter])
            assert not result.successful
        assert node.get_parameter("roi.x_min").value == -10.0
        assert node.get_parameter("landing.footprint_m").value == 1.2
        assert np.isneginf(node._landing_roi().z_min)
        # Related bounds may be updated together without an invalid interim ROI.
        result = node.set_parameters_atomically([
            Parameter("roi.x_min", value=-20.0),
            Parameter("roi.x_max", value=20.0),
            Parameter("roi.y_min", value=-20.0),
            Parameter("roi.y_max", value=20.0),
            Parameter("landing.footprint_m", value=30.0),
        ])
        assert result.successful
    finally:
        node.destroy_node()
        rclpy.shutdown()
