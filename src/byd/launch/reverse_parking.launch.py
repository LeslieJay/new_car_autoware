#!/usr/bin/env python3
"""Launch reverse parking planner and controller together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('byd_launch')
    setup_script = os.path.join(pkg_share, 'scripts', 'setup_external_control_mode.sh')

    planner_launch = os.path.join(
        get_package_share_directory('reverse_parking_planner'),
        'launch',
        'reverse_parking_planner.launch.py',
    )
    controller_launch = os.path.join(
        get_package_share_directory('reverse_parking_controller'),
        'launch',
        'reverse_parking_controller.launch.py',
    )

    declare_input_odom = DeclareLaunchArgument(
        'input_odom',
        default_value='/localization/kinematic_state',
        description='Shared odometry topic for planner and controller',
    )

    declare_trajectory = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/planning/scenario_planning/parking/trajectory',
        description='Trajectory topic from planner to controller',
    )

    declare_setup_external_control = DeclareLaunchArgument(
        'setup_external_control',
        default_value='true',
        description='Run setup script for LOCAL external control mode before parking',
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        launch_arguments={
            'input_odom': LaunchConfiguration('input_odom'),
            'output_trajectory': LaunchConfiguration('trajectory_topic'),
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch),
        launch_arguments={
            'input_odom': LaunchConfiguration('input_odom'),
            'input_trajectory': LaunchConfiguration('trajectory_topic'),
        }.items(),
    )

    setup_external_control = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', setup_script],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        declare_input_odom,
        declare_trajectory,
        declare_setup_external_control,
        planner,
        controller,
        setup_external_control,
    ])
