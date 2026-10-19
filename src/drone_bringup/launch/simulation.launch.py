"""ROS-side simulation interfaces for PX4 gz_x500_depth.

PX4/Gazebo itself is intentionally started in a separate terminal with:
    make px4_sitl gz_x500_depth
This launch then connects MAVROS, bridges the Gazebo RGB-D sensors, builds the
TF tree, creates a PointCloud2, and feeds OctoMap.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _runtime_nodes(context, *_args, **_kwargs):
    rgb_image = LaunchConfiguration('gz_rgb_image_topic').perform(context)
    rgb_info = LaunchConfiguration('gz_rgb_info_topic').perform(context)
    depth_image = LaunchConfiguration('gz_depth_image_topic').perform(context)
    depth_info = LaunchConfiguration('gz_depth_info_topic').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    bridges = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        f'{rgb_image}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{rgb_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'{depth_image}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{depth_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rgbd_gz_bridge',
        output='screen',
        arguments=bridges,
        parameters=[{'use_sim_time': use_sim_time, 'override_frame_id': 'camera_optical_frame'}],
        remappings=[
            (rgb_image, '/camera/color/image_raw'),
            (rgb_info, '/camera/color/camera_info'),
            (depth_image, '/camera/depth/image_raw'),
            (depth_info, '/camera/depth/camera_info'),
        ],
    )
    return [bridge]


def generate_launch_description():
    bringup = get_package_share_directory('drone_bringup')
    description = get_package_share_directory('drone_description')
    mavros = get_package_share_directory('mavros')
    autonomy_params = os.path.join(bringup, 'config', 'autonomy.yaml')
    octomap_params = os.path.join(bringup, 'config', 'octomap.yaml')

    # PX4's x500_depth_0 publishes RGB under the fully scoped sensor topic and
    # depth under /depth_camera. These are launch args because Gazebo model/world
    # names can change across user setups.
    default_rgb = '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image'
    default_rgb_info = '/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@'),
        DeclareLaunchArgument('gz_rgb_image_topic', default_value=default_rgb),
        DeclareLaunchArgument('gz_rgb_info_topic', default_value=default_rgb_info),
        DeclareLaunchArgument('gz_depth_image_topic', default_value='/depth_camera'),
        DeclareLaunchArgument('gz_depth_info_topic', default_value='/camera_info'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(mavros, 'launch', 'px4.launch')),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items(),
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),
        OpaqueFunction(function=_runtime_nodes),
        Node(
            package='drone_control', executable='tf_bridge', name='drone_tf_bridge',
            output='screen', parameters=[autonomy_params],
        ),
        Node(
            package='drone_perception', executable='depth_to_cloud', name='depth_to_cloud',
            output='screen', parameters=[autonomy_params],
        ),
        Node(
            package='octomap_server', executable='octomap_server_node', name='octomap_server',
            output='screen', parameters=[octomap_params],
            remappings=[('cloud_in', '/camera/depth/points')],
        ),
    ])
