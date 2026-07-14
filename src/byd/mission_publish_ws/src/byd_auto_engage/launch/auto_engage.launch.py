#!/usr/bin/env python3
"""Launch file for auto_engage_node."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import set_console_format, set_stage_log_dir  # noqa: E402


def generate_launch_description():
    pkg_share = get_package_share_directory("byd_auto_engage")
    default_config = os.path.join(pkg_share, "config", "auto_engage.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the auto engage configuration file",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level for auto_engage_node",
    )

    startup_delay_arg = DeclareLaunchArgument(
        "startup_delay_sec",
        default_value="5.0",
        description="Delay before starting auto_engage_node",
    )

    auto_engage_node = Node(
        package="byd_auto_engage",
        executable="auto_engage_node",
        name="auto_engage_node",
        output="both",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config_file")],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    delayed_auto_engage_node = TimerAction(
        period=LaunchConfiguration("startup_delay_sec"),
        actions=[auto_engage_node],
    )

    return LaunchDescription(
        [
            *set_console_format(),
            set_stage_log_dir("applications"),
            config_file_arg,
            log_level_arg,
            startup_delay_arg,
            delayed_auto_engage_node,
        ]
    )
