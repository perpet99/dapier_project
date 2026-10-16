import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------------------------------------------
    # 마커 및 배치 인자 (실제 로봇 기본값)
    # -----------------------------------------------------------------
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag Family (tag36h11, tag25h9, tag16h5)'
    )
    target_tag_id_arg = DeclareLaunchArgument(
        'target_tag_id',
        default_value='-1',
        description='Target AprilTag ID (-1 for any tag)'
    )
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.08',
        description='Physical marker outer size in meters (default: 0.08 = 8cm)'
    )
    tag_spacing_arg = DeclareLaunchArgument(
        'tag_spacing',
        default_value='0.19',
        description='Center-to-center distance between dual tags in meters (default: 0.19 = 19cm)'
    )
    is_gazebo_simulation_arg = DeclareLaunchArgument(
        'is_gazebo_simulation',
        default_value='false',
        description='Whether running inside Gazebo simulation (applies 2/3 texture margin ratio)'
    )

    # -----------------------------------------------------------------
    # 도킹 목표 및 허용오차
    # -----------------------------------------------------------------
    target_dist_arg = DeclareLaunchArgument(
        'target_dist',
        default_value='0.0',
        description='Target stopping distance in meters (default: 0.0 = complete wall contact)'
    )
    linear_tolerance_arg = DeclareLaunchArgument(
        'linear_tolerance',
        default_value='0.02',
        description='Stopping distance tolerance in meters (default: 0.02 = 2cm)'
    )
    lateral_tolerance_arg = DeclareLaunchArgument(
        'lateral_tolerance',
        default_value='0.025',
        description='Lateral alignment tolerance in meters (default: 0.025 = 2.5cm)'
    )
    angular_tolerance_arg = DeclareLaunchArgument(
        'angular_tolerance',
        default_value='0.045',
        description='Angular alignment tolerance in radians (default: 0.045 rad = ~2.5 deg)'
    )

    # -----------------------------------------------------------------
    # 속도 및 주행 인자
    # -----------------------------------------------------------------
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.15',
        description='Maximum forward linear velocity in m/s (safe for real robot)'
    )
    min_linear_speed_arg = DeclareLaunchArgument(
        'min_linear_speed',
        default_value='0.03',
        description='Minimum forward linear velocity in m/s'
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.45',
        description='Maximum angular steering velocity in rad/s'
    )
    search_angular_speed_arg = DeclareLaunchArgument(
        'search_angular_speed',
        default_value='0.35',
        description='Angular velocity during tag search or recovery in rad/s'
    )
    max_backup_count_arg = DeclareLaunchArgument(
        'max_backup_count',
        default_value='10',
        description='Maximum allowed backup attempts'
    )

    # -----------------------------------------------------------------
    # 카메라 및 센서 하드웨어 인자
    # -----------------------------------------------------------------
    use_ros_image_arg = DeclareLaunchArgument(
        'use_ros_image',
        default_value='false',
        description='Whether to subscribe to ROS 2 image topic (false: direct V4L2 USB camera)'
    )
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='V4L2 camera device path or index'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='ROS 2 Camera image topic'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='ROS 2 CameraInfo topic'
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/scan',
        description='ROS 2 LiDAR scan topic'
    )
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Enable LiDAR wall measurement (default: false for pure vision mode)'
    )
    lidar_offset_arg = DeclareLaunchArgument(
        'lidar_offset',
        default_value='0.14',
        description='Distance from LiDAR to robot front bumper/camera in meters'
    )
    flip_180_arg = DeclareLaunchArgument(
        'flip_180',
        default_value='false',
        description='Rotate camera image 180 degrees if camera is mounted upside down (default: false)'
    )
    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='true',
        description='Display live OpenCV camera GUI window on desktop screen (default: true)'
    )
    straight_drive_duration_arg = DeclareLaunchArgument(
        'straight_drive_duration',
        default_value='6.0',
        description='Duration in seconds for unconditional straight forward push when centered (default: 6.0s for complete contact)'
    )
    straight_drive_speed_arg = DeclareLaunchArgument(
        'straight_drive_speed',
        default_value='0.07',
        description='Forward speed during straight push in m/s (default: 0.07)'
    )
    straight_dock_dist_arg = DeclareLaunchArgument(
        'straight_dock_dist',
        default_value='0.25',
        description='Distance threshold in meters to trigger final straight docking push (default: 0.25 = 25cm)'
    )
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8088',
        description='Web live stream HUD port'
    )

    # -----------------------------------------------------------------
    # 노드 구성
    # -----------------------------------------------------------------
    docking_node = Node(
        package='amr_bringup',
        executable='apriltag_docking_node',
        name='apriltag_docking_node',
        parameters=[{
            'tag_family': LaunchConfiguration('tag_family'),
            'target_tag_id': LaunchConfiguration('target_tag_id'),
            'tag_size': LaunchConfiguration('tag_size'),
            'tag_spacing': LaunchConfiguration('tag_spacing'),
            'is_gazebo_simulation': LaunchConfiguration('is_gazebo_simulation'),
            'target_dist': LaunchConfiguration('target_dist'),
            'linear_tolerance': LaunchConfiguration('linear_tolerance'),
            'lateral_tolerance': LaunchConfiguration('lateral_tolerance'),
            'angular_tolerance': LaunchConfiguration('angular_tolerance'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'min_linear_speed': LaunchConfiguration('min_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'search_angular_speed': LaunchConfiguration('search_angular_speed'),
            'max_backup_count': LaunchConfiguration('max_backup_count'),
            'use_ros_image': LaunchConfiguration('use_ros_image'),
            'camera_device': LaunchConfiguration('camera_device'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'lidar_topic': LaunchConfiguration('lidar_topic'),
            'lidar_offset': LaunchConfiguration('lidar_offset'),
            'flip_180': LaunchConfiguration('flip_180'),
            'show_gui': LaunchConfiguration('show_gui'),
            'straight_drive_duration': LaunchConfiguration('straight_drive_duration'),
            'straight_drive_speed': LaunchConfiguration('straight_drive_speed'),
            'straight_dock_dist': LaunchConfiguration('straight_dock_dist'),
            'web_port': LaunchConfiguration('web_port'),
            'auto_start': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        tag_family_arg,
        target_tag_id_arg,
        tag_size_arg,
        tag_spacing_arg,
        is_gazebo_simulation_arg,
        target_dist_arg,
        linear_tolerance_arg,
        lateral_tolerance_arg,
        angular_tolerance_arg,
        max_linear_speed_arg,
        min_linear_speed_arg,
        max_angular_speed_arg,
        search_angular_speed_arg,
        max_backup_count_arg,
        use_ros_image_arg,
        camera_device_arg,
        image_topic_arg,
        camera_info_topic_arg,
        use_lidar_arg,
        lidar_topic_arg,
        lidar_offset_arg,
        flip_180_arg,
        show_gui_arg,
        straight_drive_duration_arg,
        straight_drive_speed_arg,
        straight_dock_dist_arg,
        web_port_arg,
        docking_node,
    ])
