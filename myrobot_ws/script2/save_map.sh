#!/usr/bin/env bash
set -euo pipefail

# Save the map currently being built by run_slam.sh to ~/maps/<name>.yaml + .pgm.
# Run this WHILE run_slam.sh (SLAM mapping) is still running, after driving
# the robot around enough to cover the whole space.
#
# Usage: ./save_map.sh [name]   (default name: my_map)
set +u
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
set -u

MAP_NAME="${1:-my_map}"
mkdir -p ~/maps

echo "Saving map as ~/maps/${MAP_NAME}.yaml / ~/maps/${MAP_NAME}.pgm ..."
ros2 run nav2_map_server map_saver_cli -f ~/maps/"${MAP_NAME}"

echo ""
echo "Done. Files:"
ls -la ~/maps/"${MAP_NAME}".yaml ~/maps/"${MAP_NAME}".pgm

echo ""
echo "Test navigation with this map:"
echo "  ~/gazebo/run_nav.sh ~/maps/${MAP_NAME}.yaml"
