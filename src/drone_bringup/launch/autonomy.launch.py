"""Launch mission, flight-control, exploration, perception, and safety nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('drone_bringup'), 'config', 'autonomy.yaml')
    return LaunchDescription([
        Node(package='drone_control', executable='flight_controller', name='flight_controller',
             output='screen', parameters=[params]),
        Node(package='drone_mission', executable='mission_manager', name='mission_manager',
             output='screen', parameters=[params]),
        Node(package='drone_exploration', executable='frontier_explorer', name='frontier_explorer',
             output='screen', parameters=[params]),
        Node(package='drone_perception', executable='victim_detector', name='victim_detector',
             output='screen', parameters=[params]),
        Node(package='drone_perception', executable='safety_monitor', name='safety_monitor',
             output='screen', parameters=[params]),
    ])
