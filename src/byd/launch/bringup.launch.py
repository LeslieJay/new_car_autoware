#!/usr/bin/env python3
"""Top-level staged bringup: drivers -> Autoware -> applications."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import (  # noqa: E402
    declare_bringup_arguments,
    make_wait_process,
    on_success,
    set_console_format,
)


def _build_staged_launch(context: LaunchContext):
    drivers_launch = os.path.join(
        get_package_share_directory('byd_launch'),
        'launch',
        'drivers.launch.py',
    )
    applications_launch = os.path.join(
        get_package_share_directory('byd_launch'),
        'launch',
        'applications.launch.py',
    )
    autoware_launch = os.path.join(
        get_package_share_directory('autoware_launch'),
        'launch',
        'autoware.launch.xml',
    )

    common_launch_args = {
        'map_path': LaunchConfiguration('map_path'),
        'log_level': LaunchConfiguration('log_level'),
        'log_root': LaunchConfiguration('log_root'),
        'readiness_timeout_sec': LaunchConfiguration('readiness_timeout_sec'),
        'enable_lidar': LaunchConfiguration('enable_lidar'),
        'enable_rcs': LaunchConfiguration('enable_rcs'),
        'enable_reverse_parking': LaunchConfiguration('enable_reverse_parking'),
        'enable_rosbridge': LaunchConfiguration('enable_rosbridge'),
        'enable_rviz': LaunchConfiguration('enable_rviz'),
        'driver_respawn': LaunchConfiguration('driver_respawn'),
        'rosbridge_address': LaunchConfiguration('rosbridge_address'),
        'rosbridge_port': LaunchConfiguration('rosbridge_port'),
        'can_rtk_params_file': LaunchConfiguration('can_rtk_params_file'),
        'can_driver_params_file': LaunchConfiguration('can_driver_params_file'),
        'reverse_parking_config_file': LaunchConfiguration('reverse_parking_config_file'),
        'byd_vehicle_state_config_file': LaunchConfiguration('byd_vehicle_state_config_file'),
    }

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drivers_launch),
        launch_arguments=common_launch_args.items(),
    )

    driver_topics = [
        '/rtk_imu/data_raw',
        '/vehicle/status/velocity_status',
    ]
    if context.launch_configurations.get('enable_lidar', 'true') == 'true':
        driver_topics.extend([
            '/rslidar_points',
            '/rslidar_left/points',
            '/rslidar_right/points',
        ])

    wait_drivers = make_wait_process(
        name='wait_drivers_ready',
        topics=driver_topics,
    )

    autoware = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(autoware_launch),
        launch_arguments={
            'map_path': LaunchConfiguration('map_path'),
            'rviz': LaunchConfiguration('enable_rviz'),
            'launch_sensing_driver': 'false',
        }.items(),
    )

    wait_autoware = make_wait_process(
        name='wait_autoware_ready',
        topics=['/localization/kinematic_state'],
        services=[
            '/api/operation_mode/change_to_local',
            '/api/operation_mode/enable_autoware_control',
        ],
        tfs=['base_link:imu_link', 'map:base_link'],
    )

    applications = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(applications_launch),
        launch_arguments=common_launch_args.items(),
    )

    return [
        LogInfo(msg='[bringup] stage 1/3: drivers'),
        drivers,
        wait_drivers,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_drivers,
                on_exit=on_success(
                    [
                        LogInfo(msg='[bringup] drivers ready, starting Autoware'),
                        autoware,
                        wait_autoware,
                    ],
                    'drivers',
                ),
            ),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_autoware,
                on_exit=on_success(
                    [
                        LogInfo(msg='[bringup] Autoware ready, starting applications'),
                        applications,
                    ],
                    'autoware',
                ),
            ),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        *declare_bringup_arguments(),
        *set_console_format(),
        OpaqueFunction(function=_build_staged_launch),
    ])
