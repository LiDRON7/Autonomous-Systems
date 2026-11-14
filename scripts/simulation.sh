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
# NVIDIA GPU with the container toolkit: render the camera and LiDAR on it (real time instead of a crawl).
if docker info 2>/dev/null | grep -q "Runtimes:.*nvidia"; then compose+=(-f "$simulation_root/docker-compose.gpu.yml"); fi

case "${1:-up}" in
  # The autonomy container rebuilds its workspace when src/ or the Dockerfile change (integration/entrypoint.sh).
  # If a build ever goes stale anyway: rm -rf build install log
  up) "${compose[@]}" up --build ;;
  down) "${compose[@]}" down ;;
  check) "${compose[@]}" config >/dev/null; echo "Simulation integration configuration is valid.";;
  *) echo "Usage: $0 [up|down|check]" >&2; exit 2 ;;
esac
