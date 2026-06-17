import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('reverse_parking_planner')
    
    # Config file
    config_file = os.path.join(pkg_share, 'config', 'reverse_parking_planner.param.yaml')
    
    # Declare arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the config file'
    )
    
    declare_input_odom = DeclareLaunchArgument(
        'input_odom',
        default_value='/localization/kinematic_state',
        description='Input odometry topic'
    )
    
    declare_output_trajectory = DeclareLaunchArgument(
        'output_trajectory',
        default_value='/planning/scenario_planning/parking/trajectory',
        description='Output trajectory topic'
    )
    
    # Node
    reverse_parking_planner_node = Node(
        package='reverse_parking_planner',
        executable='reverse_parking_planner_node_exe',
        name='reverse_parking_planner',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('~/input/odometry', LaunchConfiguration('input_odom')),
            ('~/output/trajectory', LaunchConfiguration('output_trajectory')),
            ('~/output/path_markers', '/planning/parking/path_markers'),
        ],
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_input_odom,
        declare_output_trajectory,
        reverse_parking_planner_node,
    ])
