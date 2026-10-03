#!/usr/bin/env bash
set -euo pipefail

autonomy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
simulation_root="${SIMULATION_ROOT:-$(cd "$autonomy_root/../Simulation" 2>/dev/null && pwd || true)}"
simulation_compose="$simulation_root/docker-compose.dev.yml"
autonomy_compose="$autonomy_root/integration/compose.autonomy.yml"

if [[ -z "$simulation_root" || ! -f "$simulation_compose" ]]; then
  echo "Set SIMULATION_ROOT to the Simulation repository checkout." >&2
  exit 2
fi

required_topics=("/oakd/depth/image" "/oakd/depth/points" "/lidar/points" "/gps")
bridge_file="$simulation_root/config/ros_gz_bridge.yaml"
for topic in "${required_topics[@]}"; do
  if ! grep -Fq "\"$topic\"" "$bridge_file"; then
    echo "Simulation bridge is missing required topic: $topic" >&2
    exit 3
  fi
done

export AUTONOMY_ROOT="$autonomy_root"
compose=(docker compose --project-directory "$simulation_root" \
  -f "$simulation_compose" -f "$autonomy_compose")

build_marker="$autonomy_root/.autonomy_build_environment"

get_environment_fingerprint() {
  {
    sha256sum \
      "$autonomy_compose" \
      "$autonomy_root/integration/Dockerfile" \
      "$autonomy_root/integration/entrypoint.sh"

    "${compose[@]}" run --rm --no-deps \
      --entrypoint bash autonomy -lc '
        echo "ROS_DISTRO=${ROS_DISTRO:-unknown}"

        dpkg-query -W \
          -f="fastcdr=${Version}\n" \
          ros-jazzy-fastcdr 2>/dev/null || true

        dpkg-query -W \
          -f="fastrtps=${Version}\n" \
          ros-jazzy-fastrtps 2>/dev/null || true

        dpkg-query -W \
          -f="rmw_fastrtps_cpp=${Version}\n" \
          ros-jazzy-rmw-fastrtps-cpp 2>/dev/null || true

        python3 --version
        cmake --version | head -n1
      '
  } | sha256sum | awk '{print $1}'
}

clean_workspace() {
  echo "Removing stale ROS build artifacts..."
  "${compose[@]}" run --rm --no-deps \
    --entrypoint bash autonomy -lc '
      rm -rf /autonomy/build /autonomy/install /autonomy/log
    '
  echo "Stale ROS build artifacts removed."
}

prepare_workspace() {
  local current
  current="$(get_environment_fingerprint)"
  echo "Current environment fingerprint: $current"

  if [[ -f "$build_marker" ]]; then
    local previous
    previous="$(cat "$build_marker")"
    if [[ "$current" != "$previous" ]]; then
      echo "Autonomy build environment has changed."
      echo "Previous fingerprint: $previous"
      echo "Current fingerprint:  $current"

      clean_workspace
    else
      echo "Existing ROS build environment is unchanged."
    fi
  elif [[ -d "$autonomy_root/build" ||
          -d "$autonomy_root/install" ||
          -d "$autonomy_root/log" ]]; then

    echo "Existing ROS build artifacts have no environment fingerprint."
    clean_workspace
  else
    echo "No existing ROS build artifacts found."
  fi
  printf '%s\n' "$current" > "${build_marker}.pending"
  rm -f "$build_marker"
}

case "${1:-up}" in
  up) echo "Building Autonomous Docker image..."
  "${compose[@]}" build autonomy
  prepare_workspace
  "${compose[@]}" up --build ;;
  down) "${compose[@]}" down ;;
  check) "${compose[@]}" config >/dev/null; echo "Simulation integration configuration is valid.";;
  *) echo "Usage: $0 [up|down|check]" >&2; exit 2 ;;
esac
