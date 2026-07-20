#!/usr/bin/env bash
set -euo pipefail

echo "Stopping Nav2 / SLAM / Gazebo / RViz / Teleop processes..."

# Nav2 launch and ROS app nodes
pkill -f "ros2 launch nav2_bringup" || true
pkill -f "tb3_simulation_launch.py" || true
pkill -f "slam_toolbox" || true
pkill -f "teleop_keyboard" || true

# GUI apps
pkill -f "rviz2" || true
pkill -f "gz sim" || true
pkill -f "gazebo" || true

sleep 1

echo "Remaining related processes (if any):"
pgrep -af "tb3_simulation_launch.py|nav2_bringup|slam_toolbox|teleop_keyboard|rviz2|gz sim|gazebo" || true

echo "Done."
