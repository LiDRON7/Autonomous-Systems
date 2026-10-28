# Autonomous Systems reference

Topics, services, parameters, tests and troubleshooting. Start with the [README](../README.md).

## Topics and services

| Package | Reads | Publishes |
| --- | --- | --- |
| `perception` | `/oakd/depth/image`, `/lidar/points` | `/perception/obstacle_detected`, `/perception/obstacle_distance`, `/landing/assessment`, `/landing/ground_points`, `/landing/non_ground_points` |
| `navigation` (mission) | `/oakd/depth/points`, `/perception/obstacle_detected`, `/landing/assessment`, `/fmu/out/vehicle_local_position`, `vehicle_attitude`, `vehicle_status`, `vehicle_command_ack` | `/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command`, `/autonomy/state`, `/autonomy/path`, `/autonomy/occupancy_grid` |
| `navigation` (diagnostics) | camera, LiDAR, `/fmu/out/vehicle_local_position`, `vehicle_status`, `vehicle_gps_position` | `/autonomy/preflight_check` service |

The mission node is the only thing that commands PX4.

| Service | Type | Does |
| --- | --- | --- |
| `/autonomy/set_local_destination` | `interfaces/srv/SetLocalDestination` | Set a north/east/down destination |
| `/autonomy/set_gps_destination` | `interfaces/srv/SetGpsDestination` | Set a latitude/longitude/altitude destination |
| `/autonomy/enable` | `std_srvs/srv/SetBool` | Start (`true`) or hold (`false`) |
| `/autonomy/abort` | `std_srvs/srv/Trigger` | Controlled landing |
| `/autonomy/preflight_check` | `std_srvs/srv/Trigger` | Check every input is live |

## Configuration

| File | Used by |
| --- | --- |
| `src/bringup/config/simulation.yaml` | `./scripts/simulation.sh` |
| `src/bringup/config/hardware.yaml` | `integration/compose.hardware.yml` |

The container picks the file from `AUTONOMY_CONFIG` (`simulation` or `hardware`). Restart it after changing parameters: `docker restart lidron_autonomy`.

LiDAR filters, all in meters and in the LiDAR frame:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `roi.x_min`, `roi.x_max`, `roi.y_min`, `roi.y_max` | `±10.0` | Keep only points inside this box. It must contain the landing footprint and clearance area. |
| `roi.z_enabled`, `roi.z_min`, `roi.z_max` | `false`, `±10.0` | Z crop. Off until the sensor orientation and flight height are checked. |
| `filters.voxel_size_m` | `0.03` | Downsampling |
| `ransac.dist_threshold` | `0.2` | Max distance from the ground plane for an inlier. Separate from the roughness limit. |
| `ransac.num_iterations` | `100` | Plane candidates (fixed seed, so results repeat) |

`landing.min_points` is also the minimum RANSAC inlier count. A small footprint returns `insufficient_ground_points`; no plane returns `ground_plane_not_found`. To see the segmentation in RViz, add PointCloud2 displays for `/landing/ground_points` and `/landing/non_ground_points` with the LiDAR frame as Fixed Frame.

## Tests

With the container running:

```bash
docker exec lidron_autonomy bash -c "
  source /opt/ros/jazzy/setup.bash && source /opt/px4_ws/install/setup.bash &&
  cd /autonomy && colcon build --symlink-install && source install/setup.bash &&
  colcon test --packages-select perception navigation && colcon test-result --verbose
"
```

The navigation tests also run without ROS, with only numpy and pytest: `cd src/navigation && python -m pytest test`.

## Troubleshooting

| Problem | Check |
| --- | --- |
| Container stopped | `docker logs lidron_autonomy` |
| No `/fmu` topics | Sim: `docker logs lidron_dds_agent`. Drone: `docker logs px4-agent`, the TELEM2 wiring and the PX4 `UXRCE_DDS_CFG` parameter. Then `ros2 topic list \| grep fmu`. |
| No camera topics | `docker logs oak-d` and `ros2 topic list \| grep oakd` |
| Topics listed but no data | Every container needs `network_mode: host`, `ipc: host` and the same `ROS_DOMAIN_ID`. |
| Simulation not found | `export SIMULATION_ROOT=/path/to/Simulation && ./scripts/simulation.sh check` |
| Rebuild the image | `docker build -f integration/Dockerfile -t lidron-autonomy:jazzy-px4-1.15 .` |
