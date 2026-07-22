#!/usr/bin/env python3
"""Launch the BYD pedestrian safety stop checker."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('byd_launch'),
        'config',
        'pedestrian_safety_stop.param.yaml',
    ])
    vehicle_info = PathJoinSubstitution([
        FindPackageShare('byd_vehicle_description'),
        'config',
        'vehicle_info.param.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_pedestrian_safety_stop',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'pedestrian_safety_stop_config_file',
            default_value=default_config,
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='autoware_surround_obstacle_checker',
            executable='autoware_surround_obstacle_checker_node',
            name='pedestrian_safety_stop',
            condition=IfCondition(
                LaunchConfiguration('enable_pedestrian_safety_stop')
            ),
            output='both',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('pedestrian_safety_stop_config_file'),
                vehicle_info,
            ],
            arguments=[
                '--ros-args',
                '--log-level',
                LaunchConfiguration('log_level'),
            ],
            remappings=[
                ('~/input/objects', '/perception/object_recognition/objects'),
                ('~/input/odometry', '/localization/kinematic_state'),
                (
                    '~/input/pointcloud',
                    '/perception/obstacle_segmentation/pointcloud',
                ),
                (
                    '~/output/max_velocity',
                    '/planning/scenario_planning/max_velocity_candidates',
                ),
                (
                    '~/output/velocity_limit_clear_command',
                    '/planning/scenario_planning/clear_velocity_limit',
                ),
                ('~/output/status', '/byd/pedestrian_safety_stop/status'),
                ('~/output/ready', '/byd/pedestrian_safety_stop/ready'),
            ],
        ),
    ])
