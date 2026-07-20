#!/usr/bin/env bash
set -euo pipefail

# TurtleBot3 map collection helper based on Nav2 getting started flow.
# Usage:
#   ./scripts/tb3_collect_map.sh start
#   ./scripts/tb3_collect_map.sh save maps/my_map
# Optional env vars:
#   MAP_TOPIC=/map
#   USE_SIM_TIME=True

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 {start|save} [map_output_prefix]"
  exit 1
fi

ACTION="$1"
MAP_PREFIX="${2:-maps/tb3_map}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
USE_SIM_TIME="${USE_SIM_TIME:-True}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set. Example: export ROS_DISTRO=jazzy"
  exit 1
fi

# ROS setup scripts can reference unset vars; temporarily disable nounset.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# For TB3 model selection (waffle, burger)
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"

case "$ACTION" in
  start)
    echo "[1/2] Starting TB3 simulation + Nav2 + SLAM"
    echo "[2/2] In another terminal, drive robot for mapping:"
    echo "      ./scripts/tb3_teleop.sh"
    ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False use_sim_time:=True
    ;;
  save)
    mkdir -p "$(dirname "$MAP_PREFIX")"
    echo "Saving map to ${MAP_PREFIX}.yaml / ${MAP_PREFIX}.pgm"

    # Capture output before grepping; piping directly into `grep -q` can close
    # the pipe before `ros2 topic list` finishes writing, causing a BrokenPipeError.
    if ! grep -qx "$MAP_TOPIC" <<< "$(ros2 topic list)"; then
      echo "Map topic not found: $MAP_TOPIC"
      echo "Run mapping first: ./scripts/tb3_collect_map.sh start"
      exit 1
    fi

    if ! timeout 8s ros2 topic echo "$MAP_TOPIC" --once >/dev/null 2>&1; then
      echo "No map message received from $MAP_TOPIC within 8s."
      echo "Check if SLAM is active and robot has moved in the map."
      exit 1
    fi

    ros2 run nav2_map_server map_saver_cli \
      -f "$MAP_PREFIX" \
      --ros-args \
      -p use_sim_time:="$USE_SIM_TIME" \
      -r map:="$MAP_TOPIC"
    ;;
  *)
    echo "Unknown action: $ACTION"
    echo "Usage: $0 {start|save} [map_output_prefix]"
    exit 1
    ;;
esac
