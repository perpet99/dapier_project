import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map_path = os.path.expanduser('~/my_map.yaml')
    default_params_path = os.path.join(pkg_amr_bringup, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    mask_arg = DeclareLaunchArgument(
        'mask',
        default_value=os.path.expanduser('~/keepout_mask.yaml'),
        description='Keepout mask yaml file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz automatically'
    )


    # 금지 구역 마스크 서버 (Filter Mask Server)
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

    # 코스트맵 필터 정보 서버 (Costmap Filter Info Server)
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

    # 필터 노드 라이프사이클 매니저
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

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False'
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False'
        }.items()
    )

    # 4. RViz2 자동 실행 (기본값 True)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        map_arg,
        mask_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        use_rviz_arg,
        filter_mask_server_node,
        costmap_filter_info_server_node,
        filter_lifecycle_manager_node,
        localization_launch,
        navigation_launch,
        rviz_launch
    ])
