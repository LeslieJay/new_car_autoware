import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("byd_system_event_monitor")
    default_params = os.path.join(
        package_share, "config", "system_event_monitor.param.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_event_monitor_param_file", default_value=default_params
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="byd_system_event_monitor",
                executable="system_event_monitor_node",
                name="system_event_monitor_node",
                output="screen",
                parameters=[LaunchConfiguration("system_event_monitor_param_file")],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            ),
        ]
    )
