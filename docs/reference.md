# Autonomous Systems reference

Parameters, tests and troubleshooting. Topics and services are in [topics.md](topics.md). Start with the [README](../README.md).

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
