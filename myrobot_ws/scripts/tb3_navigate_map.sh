#!/usr/bin/env bash
set -euo pipefail

# TurtleBot3 navigation helper with saved map.
# Usage:
#   ./scripts/tb3_navigate_map.sh start maps/tb3_map.yaml
#   ./scripts/tb3_navigate_map.sh run-goals config/nav_goals.yaml

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 {start|run-goals} <map_yaml_or_goal_yaml>"
  exit 1
fi

ACTION="$1"
INPUT_FILE="$2"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

# ROS setup scripts can reference unset vars; temporarily disable nounset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"

case "$ACTION" in
  start)
    if [[ ! -f "$INPUT_FILE" ]]; then
      echo "Map yaml not found: $INPUT_FILE"
      exit 1
    fi
    ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False map:="$INPUT_FILE" headless:=False use_sim_time:=True
    ;;
  run-goals)
    if [[ ! -f "$INPUT_FILE" ]]; then
      echo "Goal yaml not found: $INPUT_FILE"
      exit 1
    fi
    python3 src/nav_goal_runner.py --goals "$INPUT_FILE" --use-sim-time
    ;;
  *)
    echo "Unknown action: $ACTION"
    echo "Usage: $0 {start|run-goals} <map_yaml_or_goal_yaml>"
    exit 1
    ;;
esac
