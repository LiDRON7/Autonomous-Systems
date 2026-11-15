#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash

cd /autonomy
# Rebuilding takes minutes on the Pi, so only do it when the code or image recipe changed.
stamp=install/.build_stamp
current="$(find src integration/Dockerfile -type f -not -path '*/__pycache__/*' -print0 \
  | sort -z | xargs -0 sha256sum | sha256sum)"
if [[ ! -f install/setup.bash || ! -f "$stamp" || "$(cat "$stamp")" != "$current" ]]; then
  colcon build --symlink-install
  printf '%s' "$current" > "$stamp"
fi

source install/setup.bash

exec ros2 launch bringup autonomous_system.launch.py \
  config_file:="/autonomy/src/bringup/config/${AUTONOMY_CONFIG:-simulation}.yaml" \
  drone_file:="/autonomy/src/bringup/config/${DRONE:?set DRONE=widow or DRONE=huntsman}.yaml"
