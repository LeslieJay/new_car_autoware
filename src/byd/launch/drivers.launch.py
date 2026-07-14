#!/usr/bin/env python3
"""Stage 1: CAN RTK, vehicle CAN, and optional lidar drivers."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import (  # noqa: E402
    as_bool,
    declare_bringup_arguments,
    is_enabled,
    set_console_format,
    set_stage_log_dir,
)


def _launch_driver_nodes(context: LaunchContext):
    respawn = as_bool(context, "driver_respawn", default=True)
    log_level = LaunchConfiguration("log_level").perform(context)
    can_rtk_params = LaunchConfiguration("can_rtk_params_file").perform(context)
    can_driver_params = LaunchConfiguration("can_driver_params_file").perform(context)

    common = {
        "output": "both",
        "emulate_tty": True,
        "respawn": respawn,
        "respawn_delay": 2.0,
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    return [
        Node(
            package="can_six_driver",
            executable="can_rtk_node",
            name="can_six_node",
            parameters=[can_rtk_params],
            **common,
        ),
        Node(
            package="can_driver",
            executable="can_node",
            name="can_node",
            parameters=[can_driver_params],
            **common,
        ),
    ]


def generate_launch_description():
    rslidar_launch = os.path.join(
        get_package_share_directory("rslidar_sdk"),
        "launch",
        "start_3.py",
    )

    return LaunchDescription(
        [
            *declare_bringup_arguments(),
            *set_console_format(),
            set_stage_log_dir("drivers"),
            OpaqueFunction(function=_launch_driver_nodes),
            GroupAction(
                condition=IfCondition(is_enabled("enable_lidar")),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(rslidar_launch),
                    ),
                ],
            ),
        ]
    )
