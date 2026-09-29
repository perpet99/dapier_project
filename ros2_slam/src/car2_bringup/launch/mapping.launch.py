"""Map collection: bringup (camera + car2 driver + depth->scan) + slam_toolbox.

Usage:
  ros2 launch car2_bringup mapping.launch.py
Then drive the robot (teleop_twist_keyboard on /cmd_vel) around the space,
and save the map with scripts/save_map.sh once slam_toolbox has built it up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox_params.yaml')

    serial_port = LaunchConfiguration('serial_port')
    car2_api_url = LaunchConfiguration('car2_api_url')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'serial_port': serial_port,
            'car2_api_url': car2_api_url,
            'wheel_radius_m': wheel_radius_m,
            'wheel_separation_m': wheel_separation_m,
        }.items(),
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('car2_api_url', default_value=''),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.032'),
        DeclareLaunchArgument('wheel_separation_m', default_value='0.18'),
        bringup,
        slam_toolbox_node,
    ])
