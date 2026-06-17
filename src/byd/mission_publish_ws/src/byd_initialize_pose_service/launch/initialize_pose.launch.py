#!/usr/bin/env python3
"""Launch file for initialize_pose_node (C++ version)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for initialize_pose_node."""

    # Get package share directory
    pkg_share = get_package_share_directory('byd_initialize_pose_service')

    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'initial_pose.yaml')

    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the initial pose configuration file'
    )

    # Create the node with delayed startup (5 seconds) to ensure Autoware API services are ready
    initialize_pose_node = Node(
        package='byd_initialize_pose_service',
        executable='initialize_pose_node',
        name='initialize_pose_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    # Delay node startup by 5 seconds to ensure Autoware services are initialized
    delayed_initialize_pose_node = TimerAction(period=5.0, actions=[initialize_pose_node])

    return LaunchDescription([
        config_file_arg,
        delayed_initialize_pose_node,
    ])
