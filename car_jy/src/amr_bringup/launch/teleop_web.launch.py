from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    web_teleop_node = Node(
        package='amr_bringup',
        executable='web_teleop',
        name='web_teleop',
        output='screen'
    )

    return LaunchDescription([
        web_teleop_node
    ])
