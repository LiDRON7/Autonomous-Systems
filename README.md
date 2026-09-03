# LiDRON Autonomous Systems

ROS 2 Jazzy · PX4 · Docker

---

## Table of Contents

- [Overview](#overview)
- [Requirements](#requirements)
- [Setup: Linux](#setup-linux)
- [Setup: macOS](#setup-macos)
- [Setup: Windows](#setup-windows)
- [Repository Structure](#repository-structure)
- [ROS Packages and Topics](#ros-packages-and-topics)
- [How the System Works](#how-the-system-works)
- [Build](#build)
- [Run with Simulation](#run-with-simulation)
- [Use the Autonomous System](#use-the-autonomous-system)
- [Run Tests](#run-tests)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

---

## Overview

This repository contains the autonomous navigation, obstacle avoidance, and
landing software. It runs in its own ROS 2 container alongside the separate
LiDRON Simulation environment.

Gazebo worlds, vehicle models, and simulated sensors belong in the Simulation
repository and are not stored here.

The combined stack uses these containers:

| Container | Role |
| --- | --- |
| `drone_sim_dev` | Gazebo simulation |
| `drone_px4` | PX4 SITL flight controller |
| `drone_ros` | Gazebo-to-ROS bridge |
| `lidron_dds_agent` | PX4-to-ROS DDS connection |
| `lidron_autonomy` | Autonomous Systems workspace |

---

## Requirements

| Tool | Version |
| --- | --- |
| Docker Engine | 24+ |
| Docker Compose plugin | v2+ |
| Git | any |
| Ubuntu | 24.04 recommended |
| UTM | macOS only |

Clone both repositories into the same folder:

```bash
git clone https://github.com/LiDRON7/Simulation.git
git clone https://github.com/LiDRON7/Autonomous-Systems.git
```

The resulting layout should be:

```text
LiDRON/
├── Autonomous-Systems/
└── Simulation/
```

---

## Setup: Linux

Use Ubuntu 24.04 and follow the Docker and display setup in the
[Simulation README](https://github.com/LiDRON7/Simulation#setup-linux-ubuntu-2404).

Verify Docker before continuing:

```bash
docker version
docker compose version
echo $DISPLAY
```

Then open the Autonomous Systems repository:

```bash
cd Autonomous-Systems
```

---

## Setup: macOS

Run the project inside an Ubuntu 24.04 virtual machine. Gazebo should not be run
directly through Docker Desktop on macOS.

1. Install UTM.
2. Create an Ubuntu 24.04 virtual machine.
3. Install Ubuntu Desktop and Docker inside the virtual machine.
4. Clone both repositories inside the virtual machine.
5. Run every command in this README from the Ubuntu terminal.

Follow the complete VM and display instructions in the
[Simulation macOS setup](https://github.com/LiDRON7/Simulation#setup-macos-via-utm-virtual-machine).

---

## Setup: Windows

Run the project inside Ubuntu through WSL 2. Gazebo uses WSLg to display its
window on Windows.

1. Install WSL 2 with `wsl --install`.
2. Open the Ubuntu terminal.
3. Install Docker inside Ubuntu.
4. Clone both repositories inside the WSL filesystem.
5. Run every command in this README from the Ubuntu terminal.

Follow the complete installation and display instructions in the
[Simulation Windows setup](https://github.com/LiDRON7/Simulation#setup-windows-via-wsl-2).

---

## Repository Structure

```text
Autonomous-Systems/
├── src/
│   ├── interfaces/   ROS service definitions
│   ├── perception/   Sensor processing and landing evaluation
│   ├── navigation/   Mapping, route planning, and mission control
│   └── bringup/      Launch files and parameters
├── integration/      Docker and Simulation integration
└── scripts/          Simulation control script
```

---

## ROS Packages and Topics

### `interfaces`

Defines the destination services used by `navigation`:

| Service type | Fields |
| --- | --- |
| `interfaces/srv/SetLocalDestination` | North, east, and down coordinates |
| `interfaces/srv/SetGpsDestination` | Latitude, longitude, and altitude |

### `perception`

Processes camera and LiDAR data. The LiDAR pipeline removes invalid points,
downsamples the cloud with 0.03 m voxels, removes statistical outliers with a
KD-tree, and evaluates the landing surface.

Inputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/oakd/depth/image` | `sensor_msgs/msg/Image` | Immediate obstacle-distance check |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Landing-area evaluation |

Outputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/perception/obstacle_detected` | `std_msgs/msg/Bool` | Immediate obstacle warning |
| `/perception/obstacle_distance` | `std_msgs/msg/Float32` | Closest valid depth measurement |
| `/landing/assessment` | `std_msgs/msg/String` | Landing result and quality measurements as JSON |

### `navigation`

Builds the obstacle map, calculates paths, manages mission states, and owns all
PX4 flight commands.

Inputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/oakd/depth/points` | `sensor_msgs/msg/PointCloud2` | Rolling occupancy map |
| `/perception/obstacle_detected` | `std_msgs/msg/Bool` | Stop and replan trigger |
| `/landing/assessment` | `std_msgs/msg/String` | Landing approval |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | Position and NED reference |
| `/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | Vehicle heading |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | Arm and flight-mode status |
| `/fmu/out/vehicle_command_ack` | `px4_msgs/msg/VehicleCommandAck` | Command acceptance or rejection |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | Landing confirmation |

Outputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Keeps PX4 in position-control mode |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | Sends the next flight target |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Arm, mode-change, and land commands |
| `/autonomy/state` | `std_msgs/msg/String` | Current mission state |
| `/autonomy/path` | `nav_msgs/msg/Path` | Current planned path |
| `/autonomy/occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | Current obstacle map |

Services:

| Service | Type | Purpose |
| --- | --- | --- |
| `/autonomy/set_local_destination` | `interfaces/srv/SetLocalDestination` | Set a local NED destination |
| `/autonomy/set_gps_destination` | `interfaces/srv/SetGpsDestination` | Convert and set a GPS destination |
| `/autonomy/enable` | `std_srvs/srv/SetBool` | Start or hold the mission |
| `/autonomy/abort` | `std_srvs/srv/Trigger` | Request a controlled landing |
| `/autonomy/preflight_check` | `std_srvs/srv/Trigger` | Check required PX4 and sensor inputs |

### `bringup`

Starts `perception`, `navigation`, and diagnostics with either the Simulation or
hardware parameter file.

---

## How the System Works

1. The operator sets one local or GPS destination and enables the mission.
2. Diagnostics verify that PX4, camera, LiDAR, position, and GPS data are live.
3. The mission controller arms PX4, enters offboard mode, and takes off.
4. Camera depth points create a rolling occupancy grid. A* calculates a path
   from the current position toward the destination.
5. The controller follows the path through trajectory setpoints. New obstacles
   stop forward progress and trigger a new lateral route.
6. At the destination, the filtered LiDAR cloud is checked for sufficient area,
   slope, roughness, and obstacle clearance.
7. A valid assessment triggers the PX4 landing command. If the area is unsafe,
   the system checks nearby positions before aborting safely.
8. PX4 landing detection moves the mission to `COMPLETE`.

The normal mission states are:

```text
IDLE → PREFLIGHT → TAKEOFF → PLANNING → NAVIGATING
     → APPROACH → LANDING_CHECK → LANDING → COMPLETE
```

`REPLANNING`, `HOLD`, and `ABORT` handle obstacles, missing data, and failures.

---

## Build

From the `Autonomous-Systems` folder, verify that the Simulation repository can
be found:

```bash
./scripts/simulation.sh check
```

If the repositories are not next to each other, set the Simulation location:

```bash
export SIMULATION_ROOT=/absolute/path/to/Simulation
./scripts/simulation.sh check
```

Build only the Autonomous Systems image:

```bash
docker build -f integration/Dockerfile \
  -t lidron-autonomy:jazzy-px4-1.15 .
```

The first build compiles the PX4 ROS message definitions and can take several
minutes. Later builds use Docker's cache.

---

## Run with Simulation

Start Simulation and Autonomous Systems together:

```bash
./scripts/simulation.sh up
```

The command builds missing images and starts all five containers. Wait until
PX4 reports that it is ready and the `lidron_autonomy` container has completed
its ROS workspace build.

Check the running containers:

```bash
docker ps
```

Stop the complete stack:

```bash
./scripts/simulation.sh down
```

---

## Use the Autonomous System

### Open a shell

```bash
docker exec -it lidron_autonomy bash
```

Source the workspace inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash
source /autonomy/install/setup.bash
```

### Check connections

```bash
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash &&
  ros2 service call /autonomy/preflight_check std_srvs/srv/Trigger {}
"
```

Do not start a mission unless the check returns `success: true`.

### Set a destination

This example sets a local NED destination 8 meters north at an altitude of 5
meters:

```bash
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash &&
  ros2 service call /autonomy/set_local_destination \
    interfaces/srv/SetLocalDestination \
    '{north: 8.0, east: 0.0, down: -5.0}'
"
```

### Start the mission

```bash
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash &&
  ros2 service call /autonomy/enable \
    std_srvs/srv/SetBool '{data: true}'
"
```

### Monitor the mission

```bash
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash &&
  ros2 topic echo /autonomy/state
"
```

Other useful topics:

| Topic | Description |
| --- | --- |
| `/autonomy/path` | Current planned path |
| `/autonomy/occupancy_grid` | Current obstacle map |
| `/perception/obstacle_distance` | Nearest detected obstacle |
| `/landing/assessment` | Current landing evaluation |

### Abort

```bash
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash &&
  ros2 service call /autonomy/abort std_srvs/srv/Trigger {}
"
```

---

## Run Tests

Start the stack first. Then run:

```bash
docker exec lidron_autonomy bash -c "
  source /opt/ros/jazzy/setup.bash &&
  source /opt/px4_ws/install/setup.bash &&
  cd /autonomy &&
  colcon build --symlink-install &&
  source install/setup.bash &&
  colcon test --packages-select perception navigation &&
  colcon test-result --verbose
"
```

The current test suite covers perception, landing evaluation, mapping, route
planning, coordinate conversion, and mission-state safety.

---

## Configuration

| File | Use |
| --- | --- |
| `src/bringup/config/simulation.yaml` | Gazebo and PX4 SITL |
| `src/bringup/config/hardware.yaml` | Physical drone |

Restart the autonomy container after changing parameters:

```bash
docker restart lidron_autonomy
```

---

## Troubleshooting

### Simulation repository not found

```bash
export SIMULATION_ROOT=/absolute/path/to/Simulation
./scripts/simulation.sh check
```

### Autonomy container stopped

```bash
docker logs lidron_autonomy
```

### PX4 topics are missing

```bash
docker logs lidron_dds_agent
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash && ros2 topic list | grep /fmu
"
```

### Sensor topics are missing

```bash
docker exec lidron_autonomy bash -c "
  source /autonomy/install/setup.bash && ros2 topic list
"
```

### Rebuild the autonomy image

```bash
docker build -f integration/Dockerfile \
  -t lidron-autonomy:jazzy-px4-1.15 .
./scripts/simulation.sh down
./scripts/simulation.sh up
```
