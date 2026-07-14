"""Shared helpers for BYD staged bringup launch files."""

from __future__ import annotations

import os
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process.process_exited import ProcessExited
from launch.launch_context import LaunchContext as LaunchContextType
from launch.substitutions import LaunchConfiguration, PythonExpression


_TRUE_VALUES = {"1", "true", "yes", "on"}


def default_map_path() -> str:
    return os.path.join(os.environ.get("HOME", "/tmp"), "autoware_map", "3_test")


def default_log_root() -> str:
    return os.path.join(os.environ.get("HOME", "/tmp"), "autoware_log")


def wait_script_path() -> str:
    return os.path.join(
        get_package_share_directory("byd_launch"),
        "scripts",
        "wait_for_ros_ready.py",
    )


def setup_external_control_script_path() -> str:
    return os.path.join(
        get_package_share_directory("byd_launch"),
        "scripts",
        "setup_external_control_mode.sh",
    )


def as_bool(context: LaunchContextType, name: str, default: bool = False) -> bool:
    """Resolve a launch argument as a tolerant boolean."""
    try:
        value = LaunchConfiguration(name).perform(context)
    except Exception:
        return default
    return value.strip().lower() in _TRUE_VALUES


def declare_bringup_arguments() -> list[DeclareLaunchArgument]:
    vehicle_state_default = os.path.join(
        get_package_share_directory("byd_vehicle_state"),
        "config",
        "vehicle_state.param.yaml",
    )
    reverse_parking_default = os.path.join(
        get_package_share_directory("reverse_parking_planner"),
        "config",
        "reverse_parking_planner.param.yaml",
    )
    can_rtk_default = os.path.join(
        get_package_share_directory("can_six_driver"),
        "config",
        "can_params.yaml",
    )
    can_driver_default = os.path.join(
        get_package_share_directory("can_driver"),
        "config",
        "can_params.yaml",
    )

    return [
        DeclareLaunchArgument(
            "map_path",
            default_value=default_map_path(),
            description="Point cloud and lanelet2 map directory path",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Default ROS log level (debug|info|warn|error|fatal)",
        ),
        DeclareLaunchArgument(
            "log_root",
            default_value=default_log_root(),
            description="Root directory for staged log output",
        ),
        DeclareLaunchArgument(
            "readiness_timeout_sec",
            default_value="180",
            description="Timeout in seconds for each readiness stage",
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
            description="Launch three rslidar drivers",
        ),
        DeclareLaunchArgument(
            "enable_rcs",
            default_value="true",
            description="Launch agv_to_rcs stack",
        ),
        DeclareLaunchArgument(
            "enable_reverse_parking",
            default_value="true",
            description="Launch reverse parking wrapper",
        ),
        DeclareLaunchArgument(
            "enable_rosbridge",
            default_value="true",
            description="Launch rosbridge websocket server",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Launch RViz inside autoware.launch.xml",
        ),
        DeclareLaunchArgument(
            "driver_respawn",
            default_value="true",
            description="Enable controlled respawn for driver nodes",
        ),
        DeclareLaunchArgument(
            "rosbridge_address",
            default_value="127.0.0.1",
            description="rosbridge websocket bind address",
        ),
        DeclareLaunchArgument(
            "rosbridge_port",
            default_value="9090",
            description="rosbridge websocket port",
        ),
        DeclareLaunchArgument(
            "can_rtk_params_file",
            default_value=can_rtk_default,
            description="can_six_driver params file",
        ),
        DeclareLaunchArgument(
            "can_driver_params_file",
            default_value=can_driver_default,
            description="can_driver params file",
        ),
        DeclareLaunchArgument(
            "reverse_parking_config_file",
            default_value=reverse_parking_default,
            description="reverse_parking_planner config",
        ),
        DeclareLaunchArgument(
            "byd_vehicle_state_config_file",
            default_value=vehicle_state_default,
            description="byd_vehicle_state config",
        ),
    ]


def set_stage_log_dir(stage: str) -> SetEnvironmentVariable:
    return SetEnvironmentVariable(
        name="ROS_LOG_DIR",
        value=[LaunchConfiguration("log_root"), "/", stage],
    )


def set_console_format() -> list[SetEnvironmentVariable]:
    return [
        SetEnvironmentVariable(
            name="RCUTILS_CONSOLE_OUTPUT_FORMAT",
            value="[{severity}] [{time}] [{name}]: {message}",
        ),
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        SetEnvironmentVariable(name="RCUTILS_LOGGING_BUFFERED_STREAM", value="1"),
    ]


def make_wait_process(
    *,
    name: str,
    topics: Iterable[str] | None = None,
    services: Iterable[str] | None = None,
    tfs: Iterable[str] | None = None,
) -> ExecuteProcess:
    cmd = ["python3", wait_script_path()]
    for topic in topics or []:
        cmd.extend(["--topic", topic])
    for service in services or []:
        cmd.extend(["--service", service])
    for tf in tfs or []:
        cmd.extend(["--tf", tf])
    cmd.extend(["--timeout", LaunchConfiguration("readiness_timeout_sec")])
    return ExecuteProcess(cmd=cmd, output="screen", name=name)


def on_success(
    next_actions: list,
    stage_name: str,
    *,
    shutdown_on_failure: bool = True,
):
    """Continue only for exit code 0; otherwise report and optionally stop launch."""

    def _handler(event: ProcessExited, context: LaunchContextType):
        if event.returncode == 0:
            return [
                LogInfo(msg=f"[{stage_name}] readiness check passed, continuing"),
                *next_actions,
            ]

        failure_actions = [
            LogInfo(
                msg=(
                    f"[ERROR] [{stage_name}] readiness check failed "
                    f"(exit code {event.returncode})"
                )
            )
        ]
        if shutdown_on_failure:
            failure_actions.append(
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            f"{stage_name} readiness check failed "
                            f"(exit code {event.returncode})"
                        )
                    )
                )
            )
        return failure_actions

    return _handler


def register_stage_transition(
    wait_action: ExecuteProcess,
    next_actions: list,
    stage_name: str,
    *,
    shutdown_on_failure: bool = True,
) -> RegisterEventHandler:
    return RegisterEventHandler(
        OnProcessExit(
            target_action=wait_action,
            on_exit=on_success(
                next_actions,
                stage_name,
                shutdown_on_failure=shutdown_on_failure,
            ),
        )
    )


def is_enabled(condition_name: str) -> PythonExpression:
    """Case-tolerant boolean expression usable by IfCondition."""
    return PythonExpression(
        [
            "'",
            LaunchConfiguration(condition_name),
            "'.lower() in ('1', 'true', 'yes', 'on')",
        ]
    )
