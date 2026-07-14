import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'can_six_driver'
    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'can_params.yaml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the YAML parameter file.'
    )

    def check_yaml_exists(context: LaunchContext):
        params_file_path = context.launch_configurations['params_file']
        if not os.path.exists(params_file_path):
            return [LogInfo(msg=f'ERROR: Parameters file does not exist: {params_file_path}')]
        return []

    can_node = Node(
        package=package_name,
        executable='can_rtk_node',
        name='can_six_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=check_yaml_exists),
        can_node,
    ])
