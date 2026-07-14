#!/usr/bin/env python3
"""Reverse parking wrapper: wait for Autoware APIs, configure mode, start planner."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import (  # noqa: E402
    make_wait_process,
    register_stage_transition,
    setup_external_control_script_path,
    set_console_format,
)


def generate_launch_description():
    pkg_share = get_package_share_directory("reverse_parking_planner")
    default_config = os.path.join(
        pkg_share,
        "config",
        "reverse_parking_planner.param.yaml",
    )
    planner_launch = os.path.join(
        pkg_share,
        "launch",
        "reverse_parking_planner.launch.py",
    )

    wait_autoware_control = make_wait_process(
        name="wait_reverse_parking_autoware",
        services=[
            "/api/operation_mode/change_to_local",
            "/api/operation_mode/enable_autoware_control",
            "/api/autoware/set/engage",
            "/control/vehicle_cmd_gate/set_pause",
        ],
        topics=["/localization/kinematic_state"],
    )

    setup_external_control = ExecuteProcess(
        cmd=[setup_external_control_script_path()],
        output="screen",
        name="setup_external_control_mode",
    )

    reverse_parking_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        launch_arguments={
            "config_file": LaunchConfiguration("reverse_parking_config_file"),
        }.items(),
    )

    start_setup_after_wait = register_stage_transition(
        wait_autoware_control,
        [setup_external_control],
        "reverse_parking/autoware",
    )

    start_planner_after_setup = register_stage_transition(
        setup_external_control,
        [reverse_parking_planner],
        "reverse_parking/setup",
    )

    # Register exit handlers before their target processes.
    return LaunchDescription(
        [
            *set_console_format(),
            DeclareLaunchArgument(
                "reverse_parking_config_file",
                default_value=default_config,
                description="Path to reverse_parking_planner parameter file",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for reverse parking stack",
            ),
            DeclareLaunchArgument(
                "readiness_timeout_sec",
                default_value="180",
                description="Timeout for Autoware control API readiness",
            ),
            start_setup_after_wait,
            start_planner_after_setup,
            wait_autoware_control,
        ]
    )
