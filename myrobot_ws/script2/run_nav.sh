#!/usr/bin/env bash
set -euo pipefail

# One-click Nav2 navigation-test launcher for WSL:
#   loads a map saved earlier with save_map.sh and starts Nav2
#   localization (AMCL) + navigation so you can set the robot's
#   initial pose and send it a goal from RViz.
#
# Usage: ./run_nav.sh [path/to/map.yaml]   (default: ~/maps/my_map.yaml)
set +u
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL=waffle

# Use the TurtleBot3 "house" world instead of the nav2_bringup sandbox.
WORLD="/opt/ros/jazzy/share/turtlebot3_gazebo/worlds/turtlebot3_house.world"
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

MAP_YAML="${1:-$HOME/maps/my_map.yaml}"
if [ ! -f "$MAP_YAML" ]; then
	echo "[ERROR] Map file not found: $MAP_YAML"
	echo "        Build a map first with run_slam.sh (i.e. run.sh), then save it:"
	echo "          ~/gazebo/save_map.sh <name>"
	exit 1
fi

echo "[1/2] Cleaning up previous Nav2/Gazebo/RViz/teleop processes..."
pkill -f "tb3_simulation_launch.py" >/dev/null 2>&1 || true
pkill -f "ros2 launch nav2_bringup" >/dev/null 2>&1 || true
pkill -f "slam_toolbox" >/dev/null 2>&1 || true
pkill -f "teleop_keyboard" >/dev/null 2>&1 || true
pkill -f "rviz2" >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
sleep 1

# ~/.bashrc auto-detects VcXsrv and sets DISPLAY; fall back to headless if it's not running.
HEADLESS="False"
USE_RVIZ="True"
if [ -z "${DISPLAY:-}" ]; then
	echo "[WARN] DISPLAY is not set (VcXsrv not detected). Falling back to headless mode."
	echo "       RViz is required for 2D Pose Estimate / Nav2 Goal, so start VcXsrv first."
	HEADLESS="True"
	USE_RVIZ="False"
fi

echo "[2/2] Starting Nav2 navigation with map: ${MAP_YAML}"
echo "      In RViz:"
echo "        1) Click '2D Pose Estimate', then click+drag on the map where the"
echo "           robot actually is and facing, to seed AMCL localization."
echo "        2) Click 'Nav2 Goal', then click+drag at the destination to send"
echo "           the robot there autonomously."
echo ""
exec ros2 launch nav2_bringup tb3_simulation_launch.py slam:=False map:="${MAP_YAML}" headless:="${HEADLESS}" use_rviz:="${USE_RVIZ}" world:="${WORLD}"
