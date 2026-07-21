#!/usr/bin/env bash
set -euo pipefail

# Builds the myrobot_ws colcon workspace (myrobot_interfaces + myrobot_nav_service).
# Usage:
#   ./scripts/build_ws.sh

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "$WS_DIR"
colcon build --symlink-install --packages-select myrobot_interfaces myrobot_nav_service

echo "Build complete. Source the overlay with:"
echo "  source ${WS_DIR}/install/setup.bash"
