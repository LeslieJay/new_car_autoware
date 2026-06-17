import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('reverse_parking_controller')

    # Config file
    config_file = os.path.join(pkg_share, 'config', 'reverse_parking_controller.param.yaml')

    # ==================== Launch Arguments ====================
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the config file'
    )

    declare_input_trajectory = DeclareLaunchArgument(
        'input_trajectory',
        default_value='/planning/scenario_planning/parking/trajectory',
        description='Input trajectory topic from reverse parking planner'
    )

    declare_input_odom = DeclareLaunchArgument(
        'input_odom',
        default_value='/localization/kinematic_state',
        description='Input odometry topic'
    )

    declare_output_control_cmd = DeclareLaunchArgument(
        'output_control_cmd',
        default_value='/external/selected/control_cmd',
        description='Output control command topic -> vehicle_cmd_gate input/external/control_cmd'
    )

    declare_output_gear_cmd = DeclareLaunchArgument(
        'output_gear_cmd',
        default_value='/external/selected/gear_cmd',
        description='Output gear command topic -> vehicle_cmd_gate input/external/gear_cmd'
    )

    declare_output_turn_indicators_cmd = DeclareLaunchArgument(
        'output_turn_indicators_cmd',
        default_value='/external/selected/turn_indicators_cmd',
        description='Output turn indicators topic -> vehicle_cmd_gate input/external/turn_indicators_cmd'
    )

    declare_output_hazard_lights_cmd = DeclareLaunchArgument(
        'output_hazard_lights_cmd',
        default_value='/external/selected/hazard_lights_cmd',
        description='Output hazard lights topic -> vehicle_cmd_gate input/external/hazard_lights_cmd'
    )

    # ==================== Node ====================
    reverse_parking_controller_node = Node(
        package='reverse_parking_controller',
        executable='reverse_parking_controller_node_exe',
        name='reverse_parking_controller',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # 输入
            ('~/input/trajectory', LaunchConfiguration('input_trajectory')),
            ('~/input/odometry', LaunchConfiguration('input_odom')),
            # 输出 -> vehicle_cmd_gate 的 input/external 话题
            ('~/output/control_cmd', LaunchConfiguration('output_control_cmd')),
            ('~/output/gear_cmd', LaunchConfiguration('output_gear_cmd')),
            ('~/output/turn_indicators_cmd', LaunchConfiguration('output_turn_indicators_cmd')),
            ('~/output/hazard_lights_cmd', LaunchConfiguration('output_hazard_lights_cmd')),
            # 调试
            ('~/debug/markers', '/control/parking_controller/debug/markers'),
        ],
    )

    return LaunchDescription([
        declare_config_file,
        declare_input_trajectory,
        declare_input_odom,
        declare_output_control_cmd,
        declare_output_gear_cmd,
        declare_output_turn_indicators_cmd,
        declare_output_hazard_lights_cmd,
        reverse_parking_controller_node,
    ])
