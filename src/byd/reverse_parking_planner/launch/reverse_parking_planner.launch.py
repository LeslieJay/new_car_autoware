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
        description='Output trajectory topic (for debug/observability)'
    )

    declare_output_control_cmd = DeclareLaunchArgument(
        'output_control_cmd',
        default_value='/external/selected/control_cmd',
        description='Output control command topic'
    )

    declare_output_gear_cmd = DeclareLaunchArgument(
        'output_gear_cmd',
        default_value='/external/selected/gear_cmd',
        description='Output gear command topic'
    )

    declare_output_turn_indicators_cmd = DeclareLaunchArgument(
        'output_turn_indicators_cmd',
        default_value='/external/selected/turn_indicators_cmd',
        description='Output turn indicators command topic'
    )

    declare_output_hazard_lights_cmd = DeclareLaunchArgument(
        'output_hazard_lights_cmd',
        default_value='/external/selected/hazard_lights_cmd',
        description='Output hazard lights command topic'
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
            ('~/output/control_cmd', LaunchConfiguration('output_control_cmd')),
            ('~/output/gear_cmd', LaunchConfiguration('output_gear_cmd')),
            ('~/output/turn_indicators_cmd', LaunchConfiguration('output_turn_indicators_cmd')),
            ('~/output/hazard_lights_cmd', LaunchConfiguration('output_hazard_lights_cmd')),
            ('~/debug/markers', '/control/reverse_parking/debug/markers'),
        ],
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_input_odom,
        declare_output_trajectory,
        declare_output_control_cmd,
        declare_output_gear_cmd,
        declare_output_turn_indicators_cmd,
        declare_output_hazard_lights_cmd,
        reverse_parking_planner_node,
    ])
