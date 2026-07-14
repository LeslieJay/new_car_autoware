#!/usr/bin/env python3
"""Launch file for BYD mission pause/resume service node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='byd_mission_pause_service',
            executable='mission_pause_node',
            name='mission_pause_node',
            output='screen',
        ),
    ])
