# LiDRON Autonomous Systems

Unified ROS 2 autonomy for route planning, OAK-D obstacle avoidance, and
LiDAR-guided landing. The same nodes target PX4 SITL and the physical vehicle;
only launch configuration and sensor transport change.

## Safety boundary

The stack never starts a hardware mission automatically. An operator must set a
destination and explicitly enable the mission. Loss of required PX4, OAK-D, or
LiDAR data holds the vehicle and requests a controlled landing after the
configured timeout. Physical flight still requires LiDRON's bench validation,
preflight procedure, university authorization, and Part 107 supervision.

## Architecture

| Package | Responsibility |
| --- | --- |
| `lidron_interfaces` | Local-NED and GPS destination services |
| `lidron_perception` | Depth safety threshold and landing-zone assessment |
| `lidron_navigation` | Rolling map, A*, mission state machine, and PX4 commands |
| `lidron_bringup` | Shared launch file and simulation/hardware parameters |

The OAK-D point cloud is projected onto a rolling horizontal grid. Inflated
occupied cells feed A*, which produces short lateral bypass waypoints toward a
single destination. The downward LiDAR is not used for navigation. At the
destination it must confirm a sufficiently large, level, smooth, obstacle-free
surface for several frames before PX4 receives a land command.

## Run with the separate simulator

Requirements:

- Docker Engine with the Compose plugin
- The separate LiDRON `Simulation` checkout
- A working display configuration as documented by that repository

By default the script expects the repositories to be siblings:

```text
LiDRON/
├── Autonomous-Systems/
└── Simulation/
```

If they are elsewhere, export the absolute path first:

```bash
export SIMULATION_ROOT=/absolute/path/to/Simulation
```

Validate and start both repositories from this checkout:

```bash
./scripts/simulation.sh check
./scripts/simulation.sh up
```

The overlay does not copy or modify Simulation. It mounts this workspace into a
dedicated ROS container, starts the PX4-compatible Micro XRCE-DDS agent, and
adds a runtime-only downward LiDAR to Simulation's `x500_depth` model.

In another terminal, verify every required input:

```bash
docker exec lidron_autonomy bash -lc \
  'source /autonomy/install/setup.bash && \
   ros2 service call /autonomy/preflight_check std_srvs/srv/Trigger {}'
```

Set an 8 m north, 5 m high local-NED destination and enable the mission:

```bash
docker exec lidron_autonomy bash -lc \
  'source /autonomy/install/setup.bash && \
   ros2 service call /autonomy/set_local_destination \
   lidron_interfaces/srv/SetLocalDestination \
   "{north: 8.0, east: 0.0, down: -5.0}"'

docker exec lidron_autonomy bash -lc \
  'source /autonomy/install/setup.bash && \
   ros2 service call /autonomy/enable std_srvs/srv/SetBool "{data: true}"'
```

Abort at any time with:

```bash
docker exec lidron_autonomy bash -lc \
  'source /autonomy/install/setup.bash && \
   ros2 service call /autonomy/abort std_srvs/srv/Trigger {}'
```

Stop the combined stack with `./scripts/simulation.sh down`.

## Hardware transition

Build on the companion computer with ROS 2 Jazzy and the PX4 1.15
`px4_msgs` definitions, then launch with:

```bash
ros2 launch lidron_bringup autonomous_system.launch.py \
  config_file:=$(ros2 pkg prefix lidron_bringup)/share/lidron_bringup/config/hardware.yaml
```

Run `/autonomy/preflight_check` before setting a destination. The hardware
configuration uses system time, tighter freshness windows, larger obstacle
inflation, and stricter landing limits. Tune these values only from recorded
bench and controlled simulation results.

## Core interfaces

- Services: `/autonomy/set_local_destination`,
  `/autonomy/set_gps_destination`, `/autonomy/enable`, `/autonomy/abort`, and
  `/autonomy/preflight_check`
- Inputs: `/oakd/depth/image`, `/oakd/depth/points`, `/lidar/points`, and PX4
  `/fmu/out/*` state topics
- Observability: `/autonomy/state`, `/autonomy/path`,
  `/autonomy/occupancy_grid`, `/perception/obstacle_distance`, and
  `/landing/assessment`

## Development checks

Pure algorithm tests do not require ROS:

```bash
PYTHONPATH=src/lidron_perception:src/lidron_navigation \
  python3 -m pytest src/lidron_perception/test src/lidron_navigation/test
```

The integration image installs the complete test dependencies. ROS packages
are built with `colcon build --symlink-install` whenever the autonomy container
starts.
