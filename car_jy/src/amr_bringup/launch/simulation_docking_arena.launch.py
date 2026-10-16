import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('amr_bringup')
    pkg_description = get_package_share_directory('amr_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. 전용 도킹 아레나 월드 및 로봇 Xacro 경로
    world_file = os.path.join(pkg_bringup, 'worlds', 'docking_arena.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'amr_gazebo.xacro')

    # Gazebo 리소스 경로 환경 변수 설정
    models_dir = os.path.join(pkg_bringup, 'models')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{models_dir}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    # 2. 런치 인자 정의:
    # 도킹 벽면이 X=0에 있고 +X 방향을 보고 있으므로,
    # 로봇은 X=1.8m 전방에서 벽면(-X 방향, Yaw=180°=3.14159 rad)을 정면으로 바라보도록 기본 스폰
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world SDF file'
    )
    declare_x = DeclareLaunchArgument('x', default_value='1.2', description='Robot initial X (m)')
    declare_y = DeclareLaunchArgument('y', default_value='0.0', description='Robot initial Y (m)')
    declare_z = DeclareLaunchArgument('z', default_value='0.10', description='Robot initial Z (m)')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='3.14159', description='Robot initial Yaw (rad, 3.14159 = 180 deg)')

    # 3. Robot State Publisher
    robot_description = Command(['xacro ', xacro_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }]
    )

    # 4. Gazebo Sim 실행
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )

    # 5. 로봇 스폰
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'amr',
            '-string', robot_description,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ]
    )

    # 6. ROS-GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': '',
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/world/docking_arena/model/amr/link/bumper_link/sensor/bumper_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        ],
        remappings=[
            ('/world/docking_arena/model/amr/link/bumper_link/sensor/bumper_sensor/contact', '/bumper/contact')
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge
    ])
