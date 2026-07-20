#!/usr/bin/env bash
set -uo pipefail

echo "Stopping Nav2 / SLAM / Gazebo / RViz / teleop processes..."

PATTERNS=(
	"ros2 launch nav2_bringup"
	"tb3_simulation_launch.py"
	"slam_toolbox"
	"teleop_keyboard"
	"rviz2"
	"gz sim"
	"gazebo"
)

for pattern in "${PATTERNS[@]}"; do
	pkill -f "$pattern" >/dev/null 2>&1 || true
done

sleep 1

# Second pass with SIGKILL for anything that ignored SIGTERM.
for pattern in "${PATTERNS[@]}"; do
	pkill -9 -f "$pattern" >/dev/null 2>&1 || true
done

echo "Remaining related processes (if any):"
pgrep -af "tb3_simulation_launch.py|nav2_bringup|slam_toolbox|teleop_keyboard|rviz2|gz sim|gazebo" || echo "  (none)"

echo "Done."
