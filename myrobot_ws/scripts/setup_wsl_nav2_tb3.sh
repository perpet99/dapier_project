#!/usr/bin/env bash
set -euo pipefail

# Install Nav2 + TurtleBot3 simulation packages for Ubuntu/WSL.
# This follows Nav2 getting started recommendations.

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

# Install ROS 2 itself if it isn't present yet (e.g. fresh WSL Ubuntu image).
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} not found under /opt/ros/${ROS_DISTRO}. Installing..."

  sudo apt update
  sudo apt install -y curl gnupg lsb-release software-properties-common

  # Ensure the "universe" repository is enabled (required by ROS 2 apt packages).
  sudo add-apt-repository -y universe

  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME}")"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  sudo apt update
  sudo apt install -y "ros-${ROS_DISTRO}-desktop" ros-dev-tools

  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "ROS 2 ${ROS_DISTRO} installation did not produce /opt/ros/${ROS_DISTRO}/setup.bash. Aborting."
    exit 1
  fi
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
  python3-yaml \
  python3-colcon-common-extensions

# Preferred package name for Jazzy+ minimal TB3 simulation.
if ! sudo apt install -y "ros-${ROS_DISTRO}-nav2-minimal-tb3-sim"; then
  # Fallback for distro/package naming differences.
  sudo apt install -y ros-${ROS_DISTRO}-nav2-minimal-tb\*
fi

echo "Installation completed."
