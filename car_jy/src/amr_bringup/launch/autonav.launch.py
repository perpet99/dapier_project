import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_amr_bringup = get_package_share_directory('amr_bringup')

    default_map_path = os.path.expanduser('~/my_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    # 1. 하드웨어 통합 브링업 (LiDAR, 서보 모터, Odometry TF, Robot State Publisher)
    full_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch', 'full_bringup.launch.py')
        )
    )

    # 2. 내비게이션 자율주행 스택 (AMCL, Nav2, 수정된 맵 로더)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_bringup, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'autostart': 'true'
        }.items()
    )

    # 3. 웹 텔레옵 조이스틱 서버 (포트 8080)
    web_teleop_node = Node(
        package='amr_bringup',
        executable='web_teleop',
        name='web_teleop',
        parameters=[{'port': 8080}],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        full_bringup_launch,
        navigation_launch,
        web_teleop_node
    ])
