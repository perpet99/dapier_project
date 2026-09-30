"""Laptop-side RTAB-Map RGB-D mapping from the robot's remote camera streams.

The robot (Raspberry Pi) runs robot_rgbd.launch.py: car2 driver (/odom + TF),
Astra Pro depth (OpenNI2) and RGB (usb_cam). Only compressed images cross the
network; everything heavy runs here:

  /camera/color/image_raw/compressed     --republish--> /rtabmap/rgb/image
  /camera/depth_raw/image/compressedDepth --republish--> /rtabmap/depth_raw/image
  depth_image_proc/register (depth -> RGB optical frame, 320x240 -> 640x480)
  rtabmap_sync/rgbd_sync -> rtabmap_slam/rtabmap (+ wheel odometry from /odom)

Outputs: /map (2D occupancy grid, map_saver-compatible -> scripts/save_map.sh),
/rtabmap/mapData, /rtabmap/mapGraph, /rtabmap/cloud_map, TF map->odom, and the
RTAB-Map database (database_path, default ~/.ros/rtabmap.db).

Usage:
  ros2 launch car2_bringup rtabmap_mapping.launch.py                  # continue the existing database
  ros2 launch car2_bringup rtabmap_mapping.launch.py new_map:=true    # delete the database and start over
  ros2 launch car2_bringup rtabmap_mapping.launch.py use_rtabmap_viz:=true use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rtabmap.rviz')

    database_path = LaunchConfiguration('database_path')
    new_map = LaunchConfiguration('new_map')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_info_topic = LaunchConfiguration('rgb_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')

    # Everything under one namespace so the intermediate topics don't clutter /.
    ns = 'rtabmap'

    # Decompress the robot's streams once, locally, then register depth into
    # the RGB frame (the Astra Pro's RGB is a separate UVC sensor, so the
    # driver can't hand us registered depth). use_rgb_timestamp makes the
    # registered depth carry the RGB stamp, so rgbd_sync pairs them exactly.
    # Queues are deep because the Astra Pro's UVC RGB frames reach us ~0.5s
    # later than the depth frame with the same capture time (measured).
    front_end = ComposableNodeContainer(
        name='rgbd_front_end',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='image_transport',
                plugin='image_transport::Republisher',
                name='rgb_decompress',
                namespace=ns,
                parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
                # The subscriber resolves 'in' + '/<transport>' without applying
                # an 'in' remap, so remap the transport-specific topic itself.
                remappings=[('in/compressed', [rgb_topic, '/compressed']), ('out', 'rgb/image')],
            ),
            ComposableNode(
                package='image_transport',
                plugin='image_transport::Republisher',
                name='depth_decompress',
                namespace=ns,
                parameters=[{'in_transport': 'compressedDepth', 'out_transport': 'raw'}],
                remappings=[('in/compressedDepth', [depth_topic, '/compressedDepth']),
                            ('out', 'depth_raw/image')],
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::RegisterNode',
                name='register_depth',
                namespace=ns,
                parameters=[{
                    'queue_size': 30,
                    'fill_upsampling_holes': True,
                    'use_rgb_timestamp': True,
                }],
                remappings=[
                    ('depth/image_rect', 'depth_raw/image'),
                    ('depth/camera_info', depth_info_topic),
                    ('rgb/camera_info', rgb_info_topic),
                    ('depth_registered/image_rect', 'depth_registered/image'),
                    ('depth_registered/camera_info', 'depth_registered/camera_info'),
                ],
            ),
        ],
    )

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace=ns,
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', 'rgb/image'),
            ('depth/image', 'depth_registered/image'),
            ('rgb/camera_info', rgb_info_topic),
        ],
    )

    rtabmap_params = {
        'frame_id': 'base_link',
        # Empty odom_frame_id = take wheel odometry from the /odom topic and
        # sync it with the images, instead of looking up the odom->base_link TF
        # at each image stamp. When odom arrives late or irregularly from the Pi
        # the TF lookup fails outright ("extrapolation into the future"), while
        # the sync queue simply waits for the matching /odom message.
        'odom_frame_id': '',
        'map_frame_id': 'map',
        'publish_tf': True,
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'approx_sync': True,
        # /odom is 10Hz, images 10-15Hz: allow a half odom period of mismatch.
        'approx_sync_max_interval': 0.06,
        # Static camera TFs only now, but they also come from the Pi.
        'wait_for_transform': 1.0,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'database_path': database_path,
        # --- RTAB-Map core parameters (strings) ---
        'Reg/Force3DoF': 'true',          # ground robot: x, y, yaw only
        'Reg/Strategy': '0',              # visual registration
        'Vis/MinInliers': '12',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/OptimizeMaxError': '3.0',
        'Rtabmap/DetectionRate': '2',
        'Mem/IncrementalMemory': 'true',  # mapping (false = localization)
        'Grid/Sensor': '1',               # occupancy grid from depth
        'Grid/3D': 'false',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.6',           # Astra Pro is unreliable closer than ~0.6m
        'Grid/RangeMax': '4.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.6',
    }

    def rtabmap_node(arguments, condition):
        return Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace=ns,
            output='screen',
            parameters=[rtabmap_params],
            remappings=[
                ('rgbd_image', 'rgbd_image'),
                ('odom', '/odom'),
                # 2D grid on the standard /map topic so scripts/save_map.sh and
                # Nav2's map_server tooling work unchanged.
                ('map', '/map'),
            ],
            arguments=arguments,
            condition=condition,
        )

    # '-d' = delete the database on start (a fresh map).
    rtabmap_new = rtabmap_node(['-d'], IfCondition(new_map))
    rtabmap_continue = rtabmap_node([], UnlessCondition(new_map))

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=ns,
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': '',
            'subscribe_rgbd': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.06,
            'wait_for_transform': 1.0,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
        }],
        remappings=[('rgbd_image', 'rgbd_image'), ('odom', '/odom')],
        condition=IfCondition(use_rtabmap_viz),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/rtabmap.db')),
        DeclareLaunchArgument('new_map', default_value='false',
                              description='true = delete database_path and start a new map'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('rgb_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth_raw/image'),
        DeclareLaunchArgument('depth_info_topic', default_value='/camera/depth_raw/camera_info'),
        front_end,
        rgbd_sync,
        rtabmap_new,
        rtabmap_continue,
        rtabmap_viz,
        rviz,
    ])
