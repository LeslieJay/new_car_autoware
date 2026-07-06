#!/usr/bin/env python3
"""Launch file for agv_to_rcs: start 3 core nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    autoware_pose_to_rcs_pose_node = Node(
        package='agv_to_rcs',
        executable='autoware_pose_to_rcs_pose',
        name='autoware_pose_to_rcs_pose',
        output='screen',
    )

    autoware_auto_server_node = Node(
        package='agv_to_rcs',
        executable='autoware_auto_server',
        name='autoware_auto_server',
        output='screen',
    )

    main_node = Node(
        package='agv_to_rcs',
        executable='main',
        output='screen',
    )

    return LaunchDescription([
        autoware_pose_to_rcs_pose_node,
        autoware_auto_server_node,
        main_node,
    ])
