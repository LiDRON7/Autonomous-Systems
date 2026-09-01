#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

cd /autonomy
colcon build --symlink-install
source install/setup.bash

exec ros2 launch lidron_bringup autonomous_system.launch.py \
  config_file:=/autonomy/src/lidron_bringup/config/simulation.yaml
