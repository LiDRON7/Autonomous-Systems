"""Launch the complete reusable autonomy stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument("config_file"),
        Node(
            package="perception",
            executable="perception_node",
            name="perception",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="navigation",
            executable="mission_node",
            name="mission",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="navigation",
            executable="diagnostics_node",
            name="diagnostics",
            parameters=[config],
            output="screen",
        ),
    ])
