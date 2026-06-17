#!/usr/bin/env python3
"""Launch file for auto_engage_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('byd_auto_engage')
    default_config = os.path.join(pkg_share, 'config', 'auto_engage.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the auto engage configuration file',
    )

    auto_engage_node = Node(
        package='byd_auto_engage',
        executable='auto_engage_node',
        name='auto_engage_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    delayed_auto_engage_node = TimerAction(period=5.0, actions=[auto_engage_node])

    return LaunchDescription([
        config_file_arg,
        delayed_auto_engage_node,
    ])
