#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

cd /autonomy
colcon build --symlink-install

if [[ -f /autonomy/.autonomy_build_environment.pending ]]; then
  mv /autonomy/.autonomy_build_environment.pending \
    /autonomy/.autonomy_build_environment
fi

source install/setup.bash

exec ros2 launch bringup autonomous_system.launch.py \
  config_file:=/autonomy/src/bringup/config/simulation.yaml
