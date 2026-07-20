#!/usr/bin/env bash
set -euo pipefail

# Install Nav2 + TurtleBot3 simulation packages for Ubuntu/WSL.
# This follows Nav2 getting started recommendations.

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

# ROS setup scripts can reference unset vars; temporarily disable nounset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

sudo apt update
sudo apt install -y \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-turtlebot3*" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-nav2-simple-commander" \
  python3-yaml

# Preferred package name for Jazzy+ minimal TB3 simulation.
if ! sudo apt install -y "ros-${ROS_DISTRO}-nav2-minimal-tb3-sim"; then
  # Fallback for distro/package naming differences.
  sudo apt install -y ros-${ROS_DISTRO}-nav2-minimal-tb\*
fi

echo "Installation completed."
