#!/usr/bin/env python3
"""Stage 3: RCS, vehicle state, reverse parking, and rosbridge."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import (  # noqa: E402
    declare_bringup_arguments,
    is_enabled,
    set_console_format,
    set_stage_log_dir,
)


def generate_launch_description():
    vehicle_state_launch = os.path.join(
        get_package_share_directory('byd_vehicle_state'),
        'launch',
        'vehicle_state.launch.py',
    )
    agv_to_rcs_launch = os.path.join(
        get_package_share_directory('agv_to_rcs'),
        'launch',
        'agv_to_rcs.launch.py',
    )
    reverse_parking_launch = os.path.join(
        get_package_share_directory('byd_launch'),
        'launch',
        'reverse_parking.launch.py',
    )
    rosbridge_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )
    pedestrian_safety_stop_launch = os.path.join(
        get_package_share_directory('byd_launch'),
        'launch',
        'pedestrian_safety_stop.launch.py',
    )

    return LaunchDescription([
        *declare_bringup_arguments(),
        *set_console_format(),
        set_stage_log_dir('applications'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pedestrian_safety_stop_launch),
            launch_arguments={
                'enable_pedestrian_safety_stop': LaunchConfiguration(
                    'enable_pedestrian_safety_stop',
                ),
                'pedestrian_safety_stop_config_file': LaunchConfiguration(
                    'pedestrian_safety_stop_config_file',
                ),
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        ),
        GroupAction(
            condition=IfCondition(is_enabled('enable_rcs')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(agv_to_rcs_launch),
                    launch_arguments={
                        'log_level': LaunchConfiguration('log_level'),
                    }.items(),
                ),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vehicle_state_launch),
            launch_arguments={
                'byd_vehicle_state_config_file': LaunchConfiguration(
                    'byd_vehicle_state_config_file',
                ),
            }.items(),
        ),
        GroupAction(
            condition=IfCondition(is_enabled('enable_reverse_parking')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(reverse_parking_launch),
                    launch_arguments={
                        'reverse_parking_config_file': LaunchConfiguration(
                            'reverse_parking_config_file',
                        ),
                        'log_level': LaunchConfiguration('log_level'),
                        'readiness_timeout_sec': LaunchConfiguration(
                            'readiness_timeout_sec',
                        ),
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(is_enabled('enable_rosbridge')),
            actions=[
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(rosbridge_launch),
                    launch_arguments={
                        'address': LaunchConfiguration('rosbridge_address'),
                        'port': LaunchConfiguration('rosbridge_port'),
                    }.items(),
                ),
            ],
        ),
    ])
