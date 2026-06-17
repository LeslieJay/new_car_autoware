'''
Autor: wei.canming
Version: 1.0
Date: 2025-12-16 10:34:58
LastEditors: wei.canming
LastEditTime: 2025-12-16 16:32:41
Description: Launch file for mission_loop node using rclcpp_components
'''
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('mission_loop')

    params_file = os.path.join(
        pkg_share,
        'config',
        'mission_points.yaml'
    )

    container = ComposableNodeContainer(
        name='mission_loop_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mission_loop',
                plugin='autoware::mission_loop::MissionLoopNode',
                name='mission_loop',
                parameters=[params_file],
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        container
    ])
