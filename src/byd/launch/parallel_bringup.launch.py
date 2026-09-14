#!/usr/bin/env python3
"""Launch Autoware, BYD CAN drivers, lidar, and auto engage together."""

import os
import sys
from datetime import datetime

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
    default_map_path,
    make_wait_process,
    register_stage_transition,
    set_console_format,
)


def _share_path(package_name: str, *parts: str) -> PathJoinSubstitution:
    return PathJoinSubstitution([FindPackageShare(package_name), *parts])


def _set_combined_launch_log(log_directory: str, log_filename: str) -> None:
    """Redirect launch and all captured process output to one log file."""
    import launch.logging as launch_logging

    launch_config = launch_logging.launch_config
    previous_handler = launch_config.file_handlers.get("launch.log")

    # Launch creates its default handler before this launch file is loaded. Replace
    # that handler so output from the launch process is written to our session file.
    launch_config._log_dir = log_directory
    combined_handler = launch_config.log_handler_factory(
        os.path.join(log_directory, log_filename),
        encoding="utf-8",
    )
    combined_handler.setFormatter(launch_config.file_formatter)

    if previous_handler is not None:
        for logger in launch_logging.LaunchLogger.all_loggers:
            if previous_handler in logger.handlers:
                logger.removeHandler(previous_handler)
        previous_handler.close()

    launch_config.file_handlers["launch.log"] = combined_handler


def _prepare_log_directory(context: LaunchContext):
    log_root = LaunchConfiguration("log_root").perform(context)
    now = datetime.now()
    date_directory = now.strftime("%Y%m%d")
    timestamp = now.strftime("%Y%m%d%H%M%S")
    log_directory = os.path.join(log_root, date_directory)
    os.makedirs(log_directory, exist_ok=True)
    _set_combined_launch_log(log_directory, f"{timestamp}.log")

    return [
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=log_directory),
        SetEnvironmentVariable(
            name="OVERRIDE_LAUNCH_PROCESS_OUTPUT",
            value="both",
        ),
    ]


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
    can_driver_launch = _share_path("can_driver", "launch", "can_launch.py")
    rslidar_launch = _share_path("rslidar_sdk", "launch", "start_3.py")
    vehicle_state_launch = _share_path(
        "byd_vehicle_state", "launch", "vehicle_state.launch.py"
    )
    safety_launch = _share_path(
        "byd_launch", "launch", "pedestrian_safety_stop.launch.py"
    )
    event_recorder_launch = _share_path(
        "byd_event_rosbag_recorder", "launch", "event_rosbag_recorder.launch.py"
    )
    system_event_monitor_launch = _share_path(
        "byd_system_event_monitor", "launch", "system_event_monitor.launch.py"
    )

    actions = [
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(autoware_launch),
            launch_arguments={
                "map_path": LaunchConfiguration("map_path"),
                "rviz": LaunchConfiguration("enable_rviz"),
                "launch_obstacle_stop_module": LaunchConfiguration(
                    "launch_obstacle_stop_module"
                ),
                "launch_dynamic_obstacle_stop_module": LaunchConfiguration(
                    "launch_dynamic_obstacle_stop_module"
                ),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(can_driver_launch),
            launch_arguments={
                "params_file": LaunchConfiguration("can_driver_params_file"),
            }.items(),
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

    if as_bool(context, "enable_event_rosbag_recorder", default=True):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(event_recorder_launch),
                launch_arguments={
                    "event_rosbag_recorder_param_file": LaunchConfiguration(
                        "event_rosbag_recorder_param_file"
                    ),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            )
        )
    if as_bool(context, "enable_system_event_monitor", default=True):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(system_event_monitor_launch),
                launch_arguments={
                    "system_event_monitor_param_file": LaunchConfiguration(
                        "system_event_monitor_param_file"
                    ),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            )
        )

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
        shutdown_on_failure=False,
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
                default_value="/home/nvidia/autoware/log",
                description="Root directory for launch log output",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz inside autoware.launch.xml",
            ),
            DeclareLaunchArgument(
                "launch_obstacle_stop_module",
                default_value="true",
                description="Enable the static obstacle stop module",
            ),
            DeclareLaunchArgument(
                "launch_dynamic_obstacle_stop_module",
                default_value="true",
                description="Enable the dynamic obstacle stop module",
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
                "enable_event_rosbag_recorder",
                default_value="true",
                description="Launch event-triggered rolling rosbag recorder",
            ),
            DeclareLaunchArgument(
                "event_rosbag_recorder_param_file",
                default_value=_share_path(
                    "byd_event_rosbag_recorder",
                    "config",
                    "event_rosbag_recorder.param.yaml",
                ),
                description="Event rosbag recorder parameter file",
            ),
            DeclareLaunchArgument(
                "enable_system_event_monitor",
                default_value="true",
                description="Launch abnormal-stop and mode-transition event monitor",
            ),
            DeclareLaunchArgument(
                "system_event_monitor_param_file",
                default_value=_share_path(
                    "byd_system_event_monitor",
                    "config",
                    "system_event_monitor.param.yaml",
                ),
                description="System event monitor parameter file",
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
            OpaqueFunction(function=_prepare_log_directory),
            OpaqueFunction(function=_launch_everything),
        ]
    )
