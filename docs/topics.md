# Topics

Every ROS 2 topic and service the autonomy code reads or writes, who sends it and what it is for.

To see them live, open a shell in the container (`docker exec -it lidron_autonomy bash`, then `source /autonomy/install/setup.bash`) and run `ros2 topic list`, or `ros2 topic echo <topic>` for one of them.

## From the sensors

Published by the [Sensor-Control-Data-Gathering](https://github.com/LiDRON7/Sensor-Control-Data-Gathering) containers on the Pi, or by Gazebo in simulation.

| Topic | Type | Sent by | Used by | For |
| --- | --- | --- | --- | --- |
| `/oakd/depth/image` | `sensor_msgs/Image` | `oak-d` | perception ([node.py](../src/perception/perception/node.py)), diagnostics | Closest obstacle straight ahead |
| `/oakd/depth/points` | `sensor_msgs/PointCloud2` | `oak-d` | mission ([mission_node.py](../src/navigation/navigation/mission_node.py)), diagnostics | Obstacle map for path planning |
| `/lidar/points` | `sensor_msgs/PointCloud2` | `velodyne` | perception, diagnostics | Landing-zone check |

The Velodyne driver publishes `/velodyne_points`, not `/lidar/points`. When the LiDAR is mounted, set `lidar_topic: /velodyne_points` for perception in `hardware.yaml` and change the diagnostics subscription to match.

## From PX4

Published by PX4 through the `px4-agent` container on the drone, or `lidron_dds_agent` in simulation.

| Topic | Used by | For |
| --- | --- | --- |
| `/fmu/out/vehicle_local_position` | mission, diagnostics | Where the drone is (north, east, down) and the GPS home point |
| `/fmu/out/vehicle_attitude` | mission | Which way the drone is facing |
| `/fmu/out/vehicle_status` | mission, diagnostics | Armed or not, flight mode. Disarmed after landing means the mission is done. |
| `/fmu/out/vehicle_command_ack` | mission | Whether PX4 accepted a command. A rejection aborts the mission. |
| `/fmu/out/vehicle_gps_position` | diagnostics ([diagnostics_node.py](../src/navigation/navigation/diagnostics_node.py)) | GPS is alive |

## To PX4

Only the mission node sends these. Nothing else commands PX4.

| Topic | For |
| --- | --- |
| `/fmu/in/offboard_control_mode` | Keeps PX4 in offboard position control. Sent every 0.1 s while a mission is active. |
| `/fmu/in/trajectory_setpoint` | The next point to fly to |
| `/fmu/in/vehicle_command` | Arm, switch to offboard, land |

## Between the autonomy nodes

| Topic | Type | Sent by | Used by | For |
| --- | --- | --- | --- | --- |
| `/perception/obstacle_detected` | `std_msgs/Bool` | perception | mission | Stop and replan |
| `/perception/obstacle_distance` | `std_msgs/Float32` | perception | nobody yet | Closest depth, for monitoring |
| `/landing/assessment` | `std_msgs/String` (JSON) | perception | mission | Is the landing spot safe, and why not |
| `/landing/ground_points` | `sensor_msgs/PointCloud2` | perception | RViz | Ground the LiDAR found |
| `/landing/non_ground_points` | `sensor_msgs/PointCloud2` | perception | RViz | Everything else |

## For the operator

Topics published by the mission node, to watch from the laptop or RViz:

| Topic | Type | Shows |
| --- | --- | --- |
| `/autonomy/state` | `std_msgs/String` | Current mission state, like `NAVIGATING` |
| `/autonomy/path` | `nav_msgs/Path` | The planned path |
| `/autonomy/occupancy_grid` | `nav_msgs/OccupancyGrid` | The obstacle map |

Services, called from the laptop:

| Service | Type | Does |
| --- | --- | --- |
| `/autonomy/preflight_check` | `std_srvs/Trigger` | Checks every input above is live |
| `/autonomy/set_local_destination` | `interfaces/SetLocalDestination` | Sets a north/east/down goal |
| `/autonomy/set_gps_destination` | `interfaces/SetGpsDestination` | Sets a latitude/longitude/altitude goal |
| `/autonomy/enable` | `std_srvs/SetBool` | Starts (`true`) or holds (`false`) the mission |
| `/autonomy/abort` | `std_srvs/Trigger` | Lands now |

With `require_lidar: false` (the flight test), the mission and the preflight check ignore `/lidar/points` and `/landing/assessment`.
