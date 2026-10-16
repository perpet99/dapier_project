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
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    default_params_file = os.path.join(pkg_amr_bringup, 'config', 'mapper_params_online_async.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    publish_robot_state_arg = DeclareLaunchArgument(
        'publish_robot_state',
        default_value='false',
        description='Publish robot description and URDF (set to true if running without full_bringup)'
    )

    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Publish static odom->base_footprint TF (set to true only if full_bringup is NOT publishing odom TF)'
    )

    # odom -> base_footprint static TF for LiDAR-only SLAM (only if full_bringup is not publishing odom TF)
    odom_to_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        condition=IfCondition(LaunchConfiguration('publish_odom_tf'))
    )

    # URDF (robot_state_publisher) - only if publish_robot_state is true
    urdf_path = os.path.join(pkg_amr_bringup, 'urdf', 'amr.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        condition=IfCondition(LaunchConfiguration('publish_robot_state')),
        output='screen'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        publish_robot_state_arg,
        publish_odom_tf_arg,
        odom_to_base_tf_node,
        rsp_node,
        slam_toolbox_launch
    ])
