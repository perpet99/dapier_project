#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Serial port for LiDAR'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for LiDAR'
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish static TF from base_link to laser_frame'
    )
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.20',
        description='Minimum range cutoff in meters to filter out robot pillars/body'
    )

    laser_scan_dir_arg = DeclareLaunchArgument(
        'laser_scan_dir',
        default_value='true',
        description='LiDAR scan direction: true for counterclockwise, false for clockwise'
    )

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD14P'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'port_name': LaunchConfiguration('port_name')},
            {'serial_baudrate': 230400},
            {'laser_scan_dir': LaunchConfiguration('laser_scan_dir')},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
            {'range_min': LaunchConfiguration('range_min')}
        ]
    )

    # base_link to laser frame tf node (only if publish_tf is true)
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld14p',
        arguments=[
            '--x', '0.08', '--y', '0', '--z', '0.11',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('frame_id')
        ],
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )

    ld = LaunchDescription()
    ld.add_action(port_name_arg)
    ld.add_action(frame_id_arg)
    ld.add_action(publish_tf_arg)
    ld.add_action(range_min_arg)
    ld.add_action(laser_scan_dir_arg)
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)

    return ld