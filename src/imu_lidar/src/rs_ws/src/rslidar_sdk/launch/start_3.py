from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rslidar_sdk')
    config_top = os.path.join(pkg_share, 'config', 'config_top.yaml')
    config_left = os.path.join(pkg_share, 'config', 'config_left.yaml')
    config_right = os.path.join(pkg_share, 'config', 'config_right.yaml')

    top_lidar = ComposableNode(
        package='rslidar_sdk',
        plugin='robosense::lidar::RslidarSdkNode',
        name='rslidar_top_node',
        namespace='rslidar_top',
        parameters=[{'config_path': config_top}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    left_lidar = ComposableNode(
        package='rslidar_sdk',
        plugin='robosense::lidar::RslidarSdkNode',
        name='rslidar_left_node',
        namespace='rslidar_left',
        parameters=[{'config_path': config_left}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    right_lidar = ComposableNode(
        package='rslidar_sdk',
        plugin='robosense::lidar::RslidarSdkNode',
        name='rslidar_right_node',
        namespace='rslidar_right',
        parameters=[{'config_path': config_right}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='rslidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[top_lidar, left_lidar, right_lidar],
        output='screen',
    )

    return LaunchDescription([container])
