#!/usr/bin/env bash
set -euo pipefail

# Launch Nav2 TB3 simulation with SLAM + Gazebo GUI + RViz
set +u
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL=waffle

# Prevent multiple overlapping launches that can hide robot motion.
pkill -f "tb3_simulation_launch.py" >/dev/null 2>&1 || true
pkill -f "ros2 launch nav2_bringup" >/dev/null 2>&1 || true
pkill -f "teleop_keyboard" >/dev/null 2>&1 || true
pkill -f "rviz2" >/dev/null 2>&1 || true
pkill -f "gz sim -r -s" >/dev/null 2>&1 || true
sleep 1

echo "Starting teleop in a separate terminal window..."
if command -v cmd.exe >/dev/null 2>&1; then
	DISTRO="${WSL_DISTRO_NAME:-Ubuntu-24.04}"
	SCRIPT_PATH="$(cd "$(dirname "$0")" && pwd)/run_teleop.sh"
	cmd.exe /c start "TB3 Teleop" wsl.exe -d "${DISTRO}" -- bash -lc "${SCRIPT_PATH}" >/dev/null 2>&1 || true
	echo "Teleop terminal launched. Focus that terminal for keyboard input."
else
	echo "Could not auto-open a second terminal. Run teleop manually in another shell:"
	echo "  source /opt/ros/jazzy/setup.bash"
	echo "  export TURTLEBOT3_MODEL=waffle"
	echo "  ros2 run turtlebot3_teleop teleop_keyboard"
fi

HEADLESS="False"
USE_RVIZ="True"

# If DISPLAY is unavailable, GUI launch can exit immediately and leave only teleop terminal.
if [ -z "${DISPLAY:-}" ]; then
	echo "[WARN] DISPLAY is not set. Switching to headless launch."
	HEADLESS="True"
	USE_RVIZ="False"
fi

echo "Starting Nav2 simulation (slam=True, headless=${HEADLESS}, use_rviz=${USE_RVIZ})..."
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=${HEADLESS} use_rviz:=${USE_RVIZ}
