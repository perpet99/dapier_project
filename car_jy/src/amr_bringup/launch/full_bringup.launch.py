import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def find_serial_device(patterns, fallbacks):
    """하드웨어 고유 ID(/dev/serial/by-id/)로 포트를 자동 탐색하여 순서 바뀜 완벽 방지"""
    for pat in patterns:
        matches = sorted(glob.glob(f'/dev/serial/by-id/{pat}'))
        if matches:
            return os.path.realpath(matches[0])
    for fb in fallbacks:
        if os.path.exists(fb):
            return os.path.realpath(fb)
    return fallbacks[0]


def generate_launch_description():
    pkg_lidar = get_package_share_directory('ldlidar_sl_ros2')

    # 하드웨어 칩셋 기반 자동 포트 결정
    # 서보 드라이버 (QinHeng 1a86 칩셋)
    detected_servo_port = find_serial_device(
        patterns=['*1a86*', '*Single_Serial*', '*servo_port*'],
        fallbacks=['/dev/servo_port', '/dev/ttyACM0', '/dev/ttyACM1']
    )
    # 라이다 (Silicon Labs CP2102 칩셋)
    detected_lidar_port = find_serial_device(
        patterns=['*CP210*', '*Silicon_Labs*', '*lidar_port*'],
        fallbacks=['/dev/lidar_port', '/dev/ttyUSB0', '/dev/ttyUSB1']
    )

    # 1. 포트 및 구동 옵션 인자
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=detected_lidar_port,
        description='LiDAR USB Port'
    )
    servo_port_arg = DeclareLaunchArgument(
        'servo_port',
        default_value=detected_servo_port,
        description='STS3215 Servo Driver USB Port'
    )
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/ttyUSB1',
        description='ESP32 Bridge USB Port'
    )
    enable_esp32_arg = DeclareLaunchArgument(
        'enable_esp32',
        default_value='false',
        description='Enable legacy ESP32 bridge node (default: false)'
    )
    enable_button_arg = DeclareLaunchArgument(
        'enable_button',
        default_value='true',
        description='Enable physical emergency stop button node (default: true)'
    )
    button_pin_arg = DeclareLaunchArgument(
        'button_pin',
        default_value='18',
        description='GPIO BCM pin for emergency stop button (default: 18)'
    )

    invert_left_arg = DeclareLaunchArgument(
        'invert_left',
        default_value='true',
        description='Invert left wheel direction'
    )
    invert_right_arg = DeclareLaunchArgument(
        'invert_right',
        default_value='false',
        description='Invert right wheel direction'
    )
    invert_angular_arg = DeclareLaunchArgument(
        'invert_angular',
        default_value='false',
        description='Invert angular rotation direction (left/right steering)'
    )
    scale_left_arg = DeclareLaunchArgument(
        'scale_left',
        default_value='1.0',
        description='Speed scale multiplier for left wheel'
    )
    scale_right_arg = DeclareLaunchArgument(
        'scale_right',
        default_value='1.0',
        description='Speed scale multiplier for right wheel'
    )

    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.25',
        description='Minimum LiDAR range in meters to filter robot body/pillars'
    )
    laser_scan_dir_arg = DeclareLaunchArgument(
        'laser_scan_dir',
        default_value='true',
        description='LiDAR scan direction: true for CCW, false for CW'
    )

    # 2. LD14P 라이다 노드 구동
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_lidar, 'launch', 'ld14p.launch.py')),
        launch_arguments=[
            ('port_name', LaunchConfiguration('lidar_port')),
            ('frame_id', 'laser_frame'),
            ('publish_tf', 'false'),
            ('range_min', LaunchConfiguration('range_min')),
            ('laser_scan_dir', LaunchConfiguration('laser_scan_dir'))
        ]
    )

    # 3. STS3215 서보 컨트롤러 노드 구동
    servo_node = Node(
        package='amr_servo',
        executable='sts_controller',
        name='sts_controller',
        parameters=[{
            'port': LaunchConfiguration('servo_port'),
            'baud': 1000000,
            'left_id': 1,
            'right_id': 2,
            'left_rear_id': 3,
            'right_rear_id': 4,
            'four_wheel_mode': True,
            'wheel_sep': 0.30,
            'wheel_radius': 0.065,
            'wheel_mode': True,
            'invert_left': LaunchConfiguration('invert_left'),
            'invert_right': LaunchConfiguration('invert_right'),
            'invert_angular': LaunchConfiguration('invert_angular'),
            'scale_left': LaunchConfiguration('scale_left'),
            'scale_right': LaunchConfiguration('scale_right'),
            'publish_odom_tf': False,
        }],
        output='screen'
    )

    # 4. ESP32 브리지 노드 구동 (선택 옵션, 기본값: 꺼짐)
    esp32_node = Node(
        package='esp32_bridge',
        executable='esp32_bridge',
        name='esp32_bridge',
        parameters=[{
            'port': LaunchConfiguration('esp32_port'),
            'baud': 115200
        }],
        condition=IfCondition(LaunchConfiguration('enable_esp32')),
        output='screen'
    )

    # 5. Odometry TF 노드 (odom -> base_footprint)
    odom_node = Node(
        package='amr_bringup',
        executable='cmd_vel_odom',
        name='cmd_vel_odom_node',
        parameters=[{
            'invert_angular': LaunchConfiguration('invert_angular')
        }],
        output='screen'
    )

    # 6. URDF (robot_state_publisher)
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    urdf_path = os.path.join(pkg_amr_bringup, 'urdf', 'amr.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # 7. Joint State Publisher 노드 구동
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # 8. 물리 버튼 긴급 정지 노드 (BCM 18)
    button_node = Node(
        package='amr_bringup',
        executable='button_stop_node',
        name='button_stop_node',
        parameters=[{
            'pin': LaunchConfiguration('button_pin'),
            'cmd_vel_topic': '/cmd_vel',
            'web_port': 8088,
        }],
        condition=IfCondition(LaunchConfiguration('enable_button')),
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(lidar_port_arg)
    ld.add_action(servo_port_arg)
    ld.add_action(esp32_port_arg)
    ld.add_action(enable_esp32_arg)
    ld.add_action(enable_button_arg)
    ld.add_action(button_pin_arg)
    ld.add_action(invert_left_arg)
    ld.add_action(invert_right_arg)
    ld.add_action(invert_angular_arg)
    ld.add_action(scale_left_arg)
    ld.add_action(scale_right_arg)
    ld.add_action(range_min_arg)
    ld.add_action(laser_scan_dir_arg)

    ld.add_action(lidar_launch)
    ld.add_action(servo_node)
    ld.add_action(odom_node)
    ld.add_action(esp32_node)
    ld.add_action(button_node)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)

    return ld
