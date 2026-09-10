#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

cd /autonomy
colcon build --symlink-install
source install/setup.bash

exec ros2 launch bringup autonomous_system.launch.py \
  config_file:=/autonomy/src/bringup/config/simulation.yaml
