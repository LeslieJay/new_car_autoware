#!/usr/bin/env python3
"""Launch file for agv_to_rcs: start 3 core nodes with Autoware-style logging."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _agv_node(**kwargs):
    """统一三个节点的 Autoware 风格输出配置。"""
    defaults = {
        'output': 'both',          # 终端 + log 文件，与 Autoware 一致
        'emulate_tty': True,       # 保留颜色与行缓冲
        'arguments': [
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level'),
        ],
    }
    defaults.update(kwargs)
    return Node(**defaults)


def generate_launch_description():
    # Autoware / ROS2 常见控制台格式: [INFO] [time] [node]: msg
    set_console_format = SetEnvironmentVariable(
        name='RCUTILS_CONSOLE_OUTPUT_FORMAT',
        value='[{severity}] [{time}] [{name}]: {message}',
    )
    set_colorized = SetEnvironmentVariable(
        name='RCUTILS_COLORIZED_OUTPUT',
        value='1',
    )
    set_stdout_line_buffered = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_BUFFERED_STREAM',
        value='1',
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level for agv_to_rcs nodes (debug|info|warn|error|fatal)',
    )

    # 位姿节点默认 info；高频位姿已改为 DEBUG，需要时用 pose_log_level:=debug
    declare_pose_log_level = DeclareLaunchArgument(
        'pose_log_level',
        default_value='info',
        description='Log level for autoware_pose_to_rcs_pose',
    )

    autoware_pose_to_rcs_pose_node = _agv_node(
        package='agv_to_rcs',
        executable='autoware_pose_to_rcs_pose',
        name='autoware_pose_to_rcs_pose',
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('pose_log_level'),
        ],
    )

    autoware_auto_server_node = _agv_node(
        package='agv_to_rcs',
        executable='autoware_auto_server',
        name='autoware_auto_server',
    )

    # main 可执行文件内部自行创建 agv_to_rcs_main 节点，勿用 Node(name=...) 避免重名
    main_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'agv_to_rcs', 'main',
            '--ros-args', '--log-level', LaunchConfiguration('log_level'),
        ],
        output='both',
        emulate_tty=True,
        name='agv_to_rcs_main_process',
    )

    return LaunchDescription([
        set_console_format,
        set_colorized,
        set_stdout_line_buffered,
        declare_log_level,
        declare_pose_log_level,
        autoware_pose_to_rcs_pose_node,
        autoware_auto_server_node,
        main_node,
    ])
