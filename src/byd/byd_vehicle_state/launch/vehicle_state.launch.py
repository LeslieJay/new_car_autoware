import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('byd_vehicle_state')
    default_config = os.path.join(pkg_share, 'config', 'vehicle_state.param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'byd_vehicle_state_config_file',
            default_value=default_config,
        ),
        DeclareLaunchArgument(
            'input_forward_goal',
            default_value='/planning/mission_planning/goal',
            description='Forward navigation goal (PoseStamped)',
        ),
        DeclareLaunchArgument(
            'input_reverse_goal',
            default_value='/planning/parking/goal',
            description='Reverse parking goal published by reverse_parking_planner',
        ),
        DeclareLaunchArgument(
            'input_odometry',
            default_value='/localization/kinematic_state',
        ),
        DeclareLaunchArgument(
            'input_route_state',
            default_value='/api/routing/state',
        ),
        DeclareLaunchArgument(
            'input_operation_mode',
            default_value='/api/operation_mode/state',
        ),
        DeclareLaunchArgument(
            'input_lanelet_route',
            default_value='/planning/mission_planning/route',
        ),
        DeclareLaunchArgument(
            'output_state',
            default_value='/byd/autoware/state',
            description='AutowareState topic (same msg type as /autoware/state)',
        ),
        DeclareLaunchArgument(
            'output_state_name',
            default_value='/byd/autoware/state_name',
        ),
        Node(
            package='byd_vehicle_state',
            executable='vehicle_state_node_exe',
            name='byd_vehicle_state',
            output='screen',
            parameters=[LaunchConfiguration('byd_vehicle_state_config_file')],
            remappings=[
                ('~/input/forward_goal', LaunchConfiguration('input_forward_goal')),
                ('~/input/reverse_goal', LaunchConfiguration('input_reverse_goal')),
                ('~/input/odometry', LaunchConfiguration('input_odometry')),
                ('~/input/route_state', LaunchConfiguration('input_route_state')),
                ('~/input/operation_mode', LaunchConfiguration('input_operation_mode')),
                ('~/input/lanelet_route', LaunchConfiguration('input_lanelet_route')),
                ('~/output/state', LaunchConfiguration('output_state')),
                ('~/output/state_name', LaunchConfiguration('output_state_name')),
            ],
        ),
    ])
