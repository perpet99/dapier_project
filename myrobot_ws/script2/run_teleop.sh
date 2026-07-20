#!/usr/bin/env bash
set -euo pipefail

# Start TurtleBot3 keyboard teleop in this terminal.
set +u
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
set -u

export TURTLEBOT3_MODEL=waffle

# turtlebot3_teleop publishes TwistStamped on /cmd_vel unless ROS_DISTRO=="humble"
# (it checks the env var, not the actual message the bridge expects). The Gazebo
# ros_gz_bridge here still subscribes to plain Twist, so without this override
# keypresses are published but silently dropped and the robot never moves.
exec env ROS_DISTRO=humble ros2 run turtlebot3_teleop teleop_keyboard
