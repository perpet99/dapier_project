#!/usr/bin/env bash
set -euo pipefail

# Starts the go_to_pose service node (requires ./scripts/build_ws.sh to have run once,
# and Nav2 (navigate_to_pose action server) already active, e.g. via
# ./scripts/tb3_navigate_map.sh start maps/tb3_map.yaml).
# Usage:
#   ./scripts/tb3_goto_pose_service.sh

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

ros2 run myrobot_nav_service go_to_pose_service
