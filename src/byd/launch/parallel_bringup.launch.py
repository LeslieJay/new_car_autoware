#!/usr/bin/env python3
"""Launch Autoware, BYD CAN drivers, lidar, and auto engage together."""

import os
import sys

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from launch_utils import (  # noqa: E402
    as_bool,
    default_log_root,
    default_map_path,
    make_wait_process,
    register_stage_transition,
    set_console_format,
)


def _share_path(package_name: str, *parts: str) -> PathJoinSubstitution:
    return PathJoinSubstitution([FindPackageShare(package_name), *parts])


def _launch_everything(context: LaunchContext):
    respawn = as_bool(context, "driver_respawn", default=True)
    log_level = LaunchConfiguration("log_level").perform(context)

    driver_common = {
        "output": "both",
        "emulate_tty": True,
        "respawn": respawn,
        "respawn_delay": 2.0,
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    autoware_launch = _share_path("autoware_launch", "launch", "autoware.launch.xml")
    rslidar_launch = _share_path("rslidar_sdk", "launch", "start_3.py")
    vehicle_state_launch = _share_path(
        "byd_vehicle_state", "launch", "vehicle_state.launch.py"
    )
    safety_launch = _share_path(
        "byd_launch", "launch", "pedestrian_safety_stop.launch.py"
    )

    actions = [
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(autoware_launch),
            launch_arguments={
                "map_path": LaunchConfiguration("map_path"),
                "rviz": LaunchConfiguration("enable_rviz"),
                "launch_sensing_driver": "false",
            }.items(),
        ),
        Node(
            package="can_six_driver",
            executable="can_rtk_node",
            name="can_six_node",
            parameters=[LaunchConfiguration("can_rtk_params_file")],
            **driver_common,
        ),
        Node(
            package="can_driver",
            executable="can_node",
            parameters=[LaunchConfiguration("can_driver_params_file")],
            **driver_common,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rslidar_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vehicle_state_launch),
            launch_arguments={
                "byd_vehicle_state_config_file": LaunchConfiguration(
                    "byd_vehicle_state_config_file"
                ),
            }.items(),
        ),
    ]

    auto_engage = Node(
        package="byd_auto_engage",
        executable="auto_engage_node",
        name="auto_engage_node",
        output="both",
        emulate_tty=True,
        parameters=[LaunchConfiguration("auto_engage_config_file")],
        arguments=["--ros-args", "--log-level", log_level],
    )
    if not as_bool(context, "enable_pedestrian_safety_stop", default=True):
        return [*actions, auto_engage]

    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(safety_launch),
        launch_arguments={
            "enable_pedestrian_safety_stop": "true",
            "pedestrian_safety_stop_config_file": LaunchConfiguration(
                "pedestrian_safety_stop_config_file"
            ),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )
    wait_safety = make_wait_process(
        name="wait_pedestrian_safety_ready",
        topics=["/byd/pedestrian_safety_stop/ready"],
        services=["/control/vehicle_cmd_gate/set_stop"],
    )
    safety_transition = register_stage_transition(
        wait_safety,
        [auto_engage],
        "pedestrian safety stop",
    )
    return [*actions, safety_transition, safety, wait_safety]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_path",
                default_value=default_map_path(),
                description="Point cloud and lanelet2 map directory path",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Default ROS log level",
            ),
            DeclareLaunchArgument(
                "log_root",
                default_value=default_log_root(),
                description="Root directory for launch log output",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz inside autoware.launch.xml",
            ),
            DeclareLaunchArgument(
                "driver_respawn",
                default_value="true",
                description="Respawn CAN driver nodes",
            ),
            DeclareLaunchArgument(
                "readiness_timeout_sec",
                default_value="180",
                description="Timeout for pedestrian safety readiness",
            ),
            DeclareLaunchArgument(
                "enable_pedestrian_safety_stop",
                default_value="true",
                description="Stop for nearby pedestrian or unknown objects",
            ),
            DeclareLaunchArgument(
                "pedestrian_safety_stop_config_file",
                default_value=_share_path(
                    "byd_launch",
                    "config",
                    "pedestrian_safety_stop.param.yaml",
                ),
                description="pedestrian safety stop params file",
            ),
            DeclareLaunchArgument(
                "can_rtk_params_file",
                default_value=_share_path("can_six_driver", "config", "can_params.yaml"),
                description="can_six_driver params file",
            ),
            DeclareLaunchArgument(
                "can_driver_params_file",
                default_value=_share_path("can_driver", "config", "can_params.yaml"),
                description="can_driver params file",
            ),
            DeclareLaunchArgument(
                "auto_engage_config_file",
                default_value=_share_path(
                    "byd_auto_engage",
                    "config",
                    "auto_engage.yaml",
                ),
                description="byd_auto_engage params file",
            ),
            DeclareLaunchArgument(
                "byd_vehicle_state_config_file",
                default_value=_share_path(
                    "byd_vehicle_state",
                    "config",
                    "vehicle_state.param.yaml",
                ),
                description="byd_vehicle_state config",
            ),
            *set_console_format(),
            SetEnvironmentVariable(
                name="ROS_LOG_DIR",
                value=[LaunchConfiguration("log_root"), "/parallel_bringup"],
            ),
            OpaqueFunction(function=_launch_everything),
        ]
    )
