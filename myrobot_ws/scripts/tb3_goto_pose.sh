#!/usr/bin/env bash
set -euo pipefail

# Calls the go_to_pose service with a target x, y, yaw.
# Usage:
#   ./scripts/tb3_goto_pose.sh <x> <y> [yaw]

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <x> <y> [yaw]"
  exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Workspace not built yet. Run ./scripts/build_ws.sh first."
  exit 1
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u

ros2 run myrobot_nav_service go_to_pose_client "$@"
