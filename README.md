# LiDRON Autonomous Systems

ROS 2 software for autonomous navigation, obstacle avoidance, and landing.

## Organization

```text
Autonomous-Systems/
├── src/
│   ├── lidron_interfaces/   Destination service definitions
│   ├── lidron_perception/   OAK-D obstacle detection and LiDAR landing checks
│   ├── lidron_navigation/   Route planning, avoidance, and PX4 mission control
│   └── lidron_bringup/      Launch files and configuration
├── integration/             Docker and Simulation integration
└── scripts/                 Commands for starting and stopping Simulation
```

The OAK-D Pro faces forward and is used to detect obstacles and build the map.
The LiDAR faces downward and is used only to decide whether an area is safe for
landing. `lidron_navigation` connects both systems to PX4 and is the only package
that sends flight commands.

## Test the code

Start Docker, then build the integration image:

```bash
docker build -f integration/Dockerfile \
  -t lidron-autonomy:jazzy-px4-1.15 .
```

Build the ROS workspace and run all tests:

```bash
docker run --rm --entrypoint bash \
  -v "$PWD:/autonomy" \
  lidron-autonomy:jazzy-px4-1.15 -lc '
    source /opt/ros/jazzy/setup.bash
    source /opt/px4_ws/install/setup.bash
    cd /autonomy
    colcon build --symlink-install
    source install/setup.bash
    colcon test
    colcon test-result --verbose
  '
```

## Run with Simulation

Keep `Autonomous-Systems` and `Simulation` next to each other:

```text
LiDRON/
├── Autonomous-Systems/
└── Simulation/
```

Check the integration and start both repositories:

```bash
./scripts/simulation.sh check
./scripts/simulation.sh up
```

If Simulation is somewhere else, set its location first:

```bash
export SIMULATION_ROOT=/absolute/path/to/Simulation
```

Before flying, check that PX4 and every sensor are connected:

```bash
docker exec lidron_autonomy bash -lc '
  source /autonomy/install/setup.bash
  ros2 service call /autonomy/preflight_check std_srvs/srv/Trigger {}
'
```

Set a local NED destination. This example is 8 meters north at an altitude of
5 meters:

```bash
docker exec lidron_autonomy bash -lc '
  source /autonomy/install/setup.bash
  ros2 service call /autonomy/set_local_destination \
    lidron_interfaces/srv/SetLocalDestination \
    "{north: 8.0, east: 0.0, down: -5.0}"
'
```

Start the autonomous mission:

```bash
docker exec lidron_autonomy bash -lc '
  source /autonomy/install/setup.bash
  ros2 service call /autonomy/enable \
    std_srvs/srv/SetBool "{data: true}"
'
```

Abort the mission and request a controlled landing:

```bash
docker exec lidron_autonomy bash -lc '
  source /autonomy/install/setup.bash
  ros2 service call /autonomy/abort std_srvs/srv/Trigger {}
'
```

Stop everything:

```bash
./scripts/simulation.sh down
```

Parameters for Simulation are in `src/lidron_bringup/config/simulation.yaml`.
Parameters for the real drone are in `src/lidron_bringup/config/hardware.yaml`.
