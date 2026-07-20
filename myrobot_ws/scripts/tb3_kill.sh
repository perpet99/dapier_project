#!/usr/bin/env bash
set -euo pipefail

# Stop TurtleBot3/Nav2 simulation processes started by the other helper scripts.
# Usage:
#   ./scripts/tb3_kill.sh

PATTERNS=(
  "tb3_simulation_launch.py"
  "gzserver"
  "gzclient"
  "gz sim"
  "rviz2"
  "robot_state_publisher"
  "turtlebot3_teleop"
  "teleop_keyboard"
  "nav_goal_runner.py"
  "slam_toolbox"
  "nav2_"
)

FOUND=0
for PATTERN in "${PATTERNS[@]}"; do
  PIDS=$(pgrep -f "$PATTERN" || true)
  if [[ -n "$PIDS" ]]; then
    FOUND=1
    echo "Stopping (SIGINT) processes matching '$PATTERN': $PIDS"
    pkill -INT -f "$PATTERN" || true
  fi
done

if [[ "$FOUND" -eq 0 ]]; then
  echo "No matching TB3/Nav2 processes found."
  exit 0
fi

sleep 2

for PATTERN in "${PATTERNS[@]}"; do
  PIDS=$(pgrep -f "$PATTERN" || true)
  if [[ -n "$PIDS" ]]; then
    echo "Force killing (SIGKILL) remaining processes matching '$PATTERN': $PIDS"
    pkill -KILL -f "$PATTERN" || true
  fi
done

echo "Done."
