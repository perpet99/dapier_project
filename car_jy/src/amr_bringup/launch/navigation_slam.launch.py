import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    default_slam_params = os.path.join(pkg_amr_bringup, 'config', 'slam_localization_params.yaml')
    default_nav2_params = os.path.join(pkg_amr_bringup, 'config', 'nav2_params.yaml')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='SLAM Toolbox localization parameter file'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Nav2 navigation parameters file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start nav2 stack'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/my_map.yaml'),
        description='Full path to map yaml file to load'
    )

    mask_arg = DeclareLaunchArgument(
        'mask',
        default_value=os.path.expanduser('~/keepout_mask.yaml'),
        description='Keepout mask yaml file'
    )

    # 1. odom -> base_footprint 정적 TF
    odom_to_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_footprint']
    )

    # 2. SLAM Toolbox 고속 라이다 스캔 매칭 위치추정 (Scan Matching Localization)
    slam_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    # 3. Nav2 내비게이션 스택 (Planner, Controller, BT Navigator 등)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    # 4. 금지 구역 마스크 서버 (Filter Mask Server)
    filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('mask'),
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # 5. 코스트맵 필터 정보 서버 (Costmap Filter Info Server)
    costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'base': 0.0,
            'multiplier': 1.0
        }],
        output='screen'
    )

    # 6. 필터 노드 라이프사이클 매니저
    filter_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': ['filter_mask_server', 'costmap_filter_info_server']
        }],
        output='screen'
    )

    return LaunchDescription([
        slam_params_arg,
        nav2_params_arg,
        mask_arg,
        use_sim_time_arg,
        autostart_arg,
        odom_to_base_tf_node,
        slam_localization_launch,
        nav2_navigation_launch,
        filter_mask_server_node,
        costmap_filter_info_server_node,
        filter_lifecycle_manager_node
    ])
