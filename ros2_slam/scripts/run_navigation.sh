#!/usr/bin/env bash
set -euo pipefail

# Navigate a previously saved map: bringup (car2 driver + camera + depth->scan)
# + Nav2 (map_server + AMCL + planner/controller).
#
# Usage: ./scripts/run_navigation.sh maps/my_map.yaml [serial_port | http://<car-ip>:8766]

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <map.yaml> [serial_port | http://<car-ip>:8766]"
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
MAP_YAML="$1"

if [[ ! -f "$MAP_YAML" ]]; then
  echo "Map file not found: $MAP_YAML"
  exit 1
fi

# Car connection: a local serial port (default /dev/ttyUSB0), or a car2_web.py
# REST API URL (http://...) to drive the car remotely. CAR2_API_URL in the
# environment is used when no target is given on the command line.
CAR2_TARGET="${2:-${CAR2_API_URL:-/dev/ttyUSB0}}"
if [[ "$CAR2_TARGET" == http://* || "$CAR2_TARGET" == https://* ]]; then
  CAR2_ARGS=(car2_api_url:="${CAR2_TARGET}")
  echo "car2 driver: REST API ${CAR2_TARGET}"
else
  CAR2_ARGS=(serial_port:="${CAR2_TARGET}")
  echo "car2 driver: serial ${CAR2_TARGET}"
fi

# Avoid colliding with other ROS2 traffic on the default domain (0) or any
# domain left with stale/leftover participants from earlier sessions.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "In RViz:"
echo "  1) '2D Pose Estimate' -> click+drag where the robot actually is/faces."
echo "  2) 'Nav2 Goal' -> click+drag at the destination."
echo ""

exec ros2 launch car2_bringup navigation.launch.py \
  map:="${MAP_YAML}" "${CAR2_ARGS[@]}"
