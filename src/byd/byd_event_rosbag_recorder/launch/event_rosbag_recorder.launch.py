from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("byd_event_rosbag_recorder")
    default_params = os.path.join(
        package_share, "config", "event_rosbag_recorder.param.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "event_rosbag_recorder_param_file", default_value=default_params
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            Node(
                package="byd_event_rosbag_recorder",
                executable="event_rosbag_recorder_node",
                name="event_rosbag_recorder_node",
                output="screen",
                parameters=[LaunchConfiguration("event_rosbag_recorder_param_file")],
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            ),
        ]
    )
