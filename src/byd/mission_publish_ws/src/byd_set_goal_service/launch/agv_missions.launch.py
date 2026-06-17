#!/usr/bin/env python3
"""Combined launch file for AGV mission services (initialize_pose and set_goal)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for AGV mission services."""

    # Get package share directories
    initialize_pose_pkg_share = get_package_share_directory('byd_initialize_pose_service')
    set_goal_pkg_share = get_package_share_directory('byd_set_goal_service')

    # Default config file paths
    default_initialize_pose_config = os.path.join(initialize_pose_pkg_share, 'config', 'initial_pose.yaml')
    default_set_goal_config = os.path.join(set_goal_pkg_share, 'config', 'goal_points.yaml')

    # Declare launch arguments for config files
    initialize_pose_config_arg = DeclareLaunchArgument(
        'initialize_pose_config',
        default_value=default_initialize_pose_config,
        description='Path to the initial pose configuration file'
    )

    set_goal_config_arg = DeclareLaunchArgument(
        'set_goal_config',
        default_value=default_set_goal_config,
        description='Path to the goal points configuration file'
    )

    # Create initialize_pose_node
    initialize_pose_node = Node(
        package='byd_initialize_pose_service',
        executable='initialize_pose_node',
        name='initialize_pose_node',
        output='screen',
        parameters=[LaunchConfiguration('initialize_pose_config')],
    )

    # Create set_goal_node
    set_goal_node = Node(
        package='byd_set_goal_service',
        executable='set_goal_node',
        name='set_goal_node',
        output='screen',
        parameters=[LaunchConfiguration('set_goal_config')],
    )

    # Delay node startup by 5 seconds to ensure Autoware services are initialized
    delayed_initialize_pose_node = TimerAction(period=5.0, actions=[initialize_pose_node])
    delayed_set_goal_node = TimerAction(period=6.0, actions=[set_goal_node])

    return LaunchDescription([
        initialize_pose_config_arg,
        set_goal_config_arg,
        delayed_initialize_pose_node,
        delayed_set_goal_node,
    ])
