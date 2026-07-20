#!/usr/bin/env bash
set -euo pipefail

# One-click Nav2 (docs.nav2.org getting-started) launcher for WSL:
#   1) clean up stale Nav2/Gazebo/RViz/teleop processes
#   2) auto-open a second WSL terminal running keyboard teleop
#   3) start the TB3 SLAM simulation in this terminal
set +u
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL=waffle

# Use the TurtleBot3 "house" world instead of the nav2_bringup sandbox.
WORLD="/opt/ros/jazzy/share/turtlebot3_gazebo/worlds/turtlebot3_house.world"
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

echo "[1/3] Cleaning up previous Nav2/Gazebo/RViz/teleop processes..."
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
	HEADLESS="True"
	USE_RVIZ="False"
fi

echo "[2/3] Opening a second WSL terminal for keyboard teleop..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
if command -v cmd.exe >/dev/null 2>&1; then
	DISTRO="${WSL_DISTRO_NAME:-Ubuntu-24.04}"
	# stdin must be detached (</dev/null) or cmd.exe/wsl.exe can try to read the
	# controlling tty and get stopped (T state) by job control, hanging this script.
	setsid cmd.exe /c start "TB3 Teleop" wsl.exe -d "${DISTRO}" -- bash -lc "${SCRIPT_DIR}/run_teleop.sh" </dev/null >/dev/null 2>&1 &
	disown
	echo "      Teleop terminal launched. Click that window, then use i/j/l/, to move and k to stop."
else
	echo "      Could not auto-open a terminal. Run this manually in another shell:"
	echo "        ${SCRIPT_DIR}/run_teleop.sh"
fi

echo "[3/3] Starting Nav2 SLAM simulation (headless=${HEADLESS}, use_rviz=${USE_RVIZ})..."
echo "      Drive the robot around with the teleop terminal to build the map."
echo "      When done, Ctrl+C here and save the map with:"
echo "        mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map"
echo ""
exec ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:="${HEADLESS}" use_rviz:="${USE_RVIZ}" world:="${WORLD}"
