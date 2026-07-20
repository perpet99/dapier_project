#!/usr/bin/env bash
set -euo pipefail

# TurtleBot3 teleop helper for WSL/ROS2.
# Usage:
#   ./scripts/tb3_teleop.sh
#   TURTLEBOT3_MODEL=burger ./scripts/tb3_teleop.sh

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

# ROS setup scripts can reference unset vars; temporarily disable nounset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"

echo "Running teleop with TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"

# Capture output before grepping; piping directly into `grep -q` can close
# the pipe before `ros2 topic list` finishes writing, causing a BrokenPipeError.
TOPIC_LIST="$(ros2 topic list)"

if ! grep -qx "/cmd_vel" <<< "$TOPIC_LIST"; then
  echo "[WARN] /cmd_vel topic is not visible yet."
  echo "       Start simulation first: ./scripts/tb3_collect_map.sh start"
else
  SUBS=$(ros2 topic info /cmd_vel 2>/dev/null | awk -F': ' '/Subscriber count/ {print $2}')
  if [[ -n "${SUBS:-}" && "$SUBS" == "0" ]]; then
    echo "[WARN] /cmd_vel has 0 subscribers. Robot will not move."
    echo "       Check Gazebo is running and robot bridge/controller is active."
  fi
fi

if grep -qx "/clock" <<< "$TOPIC_LIST"; then
  if ! timeout 3s ros2 topic echo /clock --once >/dev/null 2>&1; then
    echo "[WARN] /clock topic exists but no messages received."
    echo "       Gazebo might be paused. Press Play in Gazebo."
  fi
fi

echo "Tip: click this terminal before pressing keys."

# turtlebot3_teleop's teleop_keyboard publishes TwistStamped on /cmd_vel unless
# ROS_DISTRO is exactly "humble" (it then publishes plain Twist). This Nav2/Gazebo
# simulation's bridge subscribes to /cmd_vel as plain Twist, so a TwistStamped
# publisher never connects to it and key presses silently do nothing. Overriding
# ROS_DISTRO here only affects this one process's message-type branch; the ROS
# environment was already sourced above from the real ROS_DISTRO.
ROS_DISTRO=humble ros2 run turtlebot3_teleop teleop_keyboard
