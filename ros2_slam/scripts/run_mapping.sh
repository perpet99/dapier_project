#!/usr/bin/env bash
set -euo pipefail

# Start hardware bringup (car2 driver + depth camera + depth->laserscan) and
# slam_toolbox mapping. Drive the robot around with teleop in another
# terminal, then run ./scripts/save_map.sh <name> once the map looks complete.
#
# Usage: ./scripts/run_mapping.sh [serial_port | http://<car-ip>:8766]

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
# Car connection: a local serial port (default /dev/ttyUSB0), or a car2_web.py
# REST API URL (http://...) to drive the car remotely. CAR2_API_URL in the
# environment is used when no target is given on the command line.
CAR2_TARGET="${1:-${CAR2_API_URL:-/dev/ttyUSB0}}"
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

echo "Driving the robot: in another terminal run (same ROS_DOMAIN_ID=${ROS_DOMAIN_ID}!)"
echo "  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

exec ros2 launch car2_bringup mapping.launch.py "${CAR2_ARGS[@]}"
