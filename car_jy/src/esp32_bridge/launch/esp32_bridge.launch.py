import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/esp32_debug',
        description='ESP32 serial port device path'
    )
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='ESP32 serial baudrate'
    )
    wheel_sep_arg = DeclareLaunchArgument(
        'wheel_sep',
        default_value='0.0',
        description='Wheel separation distance (m)'
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0',
        description='Wheel radius (m)'
    )

    esp32_bridge_node = Node(
        package='esp32_bridge',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'wheel_sep': LaunchConfiguration('wheel_sep'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        wheel_sep_arg,
        wheel_radius_arg,
        esp32_bridge_node
    ])
