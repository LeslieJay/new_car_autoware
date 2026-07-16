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
)
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import (  # noqa: E402
    as_bool,
    declare_bringup_arguments,
    make_wait_process,
    register_stage_transition,
    set_console_format,
)


def _build_staged_launch(context: LaunchContext):
    byd_share = get_package_share_directory("byd_launch")

    drivers_launch = os.path.join(byd_share, "launch", "drivers.launch.py")
    applications_launch = os.path.join(byd_share, "launch", "applications.launch.py")
    auto_engage_launch = os.path.join(
        get_package_share_directory("byd_auto_engage"),
        "launch",
        "auto_engage.launch.py",
    )
    autoware_launch = os.path.join(
        get_package_share_directory("autoware_launch"),
        "launch",
        "autoware.launch.xml",
    )

    common_launch_args = {
        "map_path": LaunchConfiguration("map_path"),
        "log_level": LaunchConfiguration("log_level"),
        "log_root": LaunchConfiguration("log_root"),
        "readiness_timeout_sec": LaunchConfiguration("readiness_timeout_sec"),
        "enable_lidar": LaunchConfiguration("enable_lidar"),
        "enable_rcs": LaunchConfiguration("enable_rcs"),
        "enable_reverse_parking": LaunchConfiguration("enable_reverse_parking"),
        "enable_rosbridge": LaunchConfiguration("enable_rosbridge"),
        "enable_rviz": LaunchConfiguration("enable_rviz"),
        "driver_respawn": LaunchConfiguration("driver_respawn"),
        "rosbridge_address": LaunchConfiguration("rosbridge_address"),
        "rosbridge_port": LaunchConfiguration("rosbridge_port"),
        "can_rtk_params_file": LaunchConfiguration("can_rtk_params_file"),
        "can_driver_params_file": LaunchConfiguration("can_driver_params_file"),
        "reverse_parking_config_file": LaunchConfiguration(
            "reverse_parking_config_file"
        ),
        "byd_vehicle_state_config_file": LaunchConfiguration(
            "byd_vehicle_state_config_file"
        ),
    }

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(drivers_launch),
        launch_arguments=common_launch_args.items(),
    )

    driver_topics = [
        "/rtk_imu/data_raw",
        "/vehicle/status/velocity_status",
    ]
    if as_bool(context, "enable_lidar", default=True):
        driver_topics.extend(
            [
                "/rslidar_points",
                "/rslidar_left/points",
                "/rslidar_right/points",
            ]
        )

    wait_drivers = make_wait_process(
        name="wait_drivers_ready",
        topics=driver_topics,
    )

    autoware = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(autoware_launch),
        launch_arguments={
            "map_path": LaunchConfiguration("map_path"),
            "rviz": LaunchConfiguration("enable_rviz"),
            "launch_sensing_driver": "false",
        }.items(),
    )

    # The log shows base_link->imu_link is published shortly after
    # robot_state_publisher starts. map->base_link requires localization to
    # initialize, so it remains a valid final readiness requirement.
    wait_autoware = make_wait_process(
        name="wait_autoware_ready",
        topics=["/localization/kinematic_state"],
        services=[
            "/api/operation_mode/change_to_local",
            "/api/operation_mode/enable_autoware_control",
        ],
        tfs=["base_link:imu_link", "map:base_link"],
    )

    applications = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(applications_launch),
        launch_arguments=common_launch_args.items(),
    )

    # auto_engage consumes Autoware's routing and operation-mode APIs, so defer
    # it until the Autoware readiness gate has passed along with stage 3 apps.
    auto_engage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(auto_engage_launch),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    autoware_transition = register_stage_transition(
        wait_drivers,
        [
            LogInfo(msg="[bringup] stage 2/3: Autoware"),
            autoware,
            wait_autoware,
        ],
        "drivers",
    )

    applications_transition = register_stage_transition(
        wait_autoware,
        [
            LogInfo(msg="[bringup] stage 3/3: applications"),
            applications,
            auto_engage,
            LogInfo(msg="[bringup] staged startup completed"),
        ],
        "autoware",
    )

    # Register handlers before starting their target processes.
    return [
        LogInfo(msg="[bringup] stage 1/3: drivers"),
        autoware_transition,
        applications_transition,
        drivers,
        wait_drivers,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *declare_bringup_arguments(),
            *set_console_format(),
            OpaqueFunction(function=_build_staged_launch),
        ]
    )
