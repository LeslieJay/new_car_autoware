# Copyright 2026 BYD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agv_high_precision_reverse_controller')
    default_config = os.path.join(
        share, 'config', 'agv_high_precision_reverse_controller.param.yaml')

    arguments = [
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument(
            'input_odom', default_value='/localization/kinematic_state'),
        DeclareLaunchArgument(
            'input_rear_warning_level', default_value='/control/rear_warning_level'),
        DeclareLaunchArgument(
            'output_trajectory',
            default_value='/planning/scenario_planning/parking/trajectory'),
        DeclareLaunchArgument(
            'output_control_cmd', default_value='/external/selected/control_cmd'),
        DeclareLaunchArgument(
            'output_gear_cmd', default_value='/external/selected/gear_cmd'),
        DeclareLaunchArgument(
            'output_turn_indicators_cmd',
            default_value='/external/selected/turn_indicators_cmd'),
        DeclareLaunchArgument(
            'output_hazard_lights_cmd',
            default_value='/external/selected/hazard_lights_cmd'),
    ]

    node = Node(
        package='agv_high_precision_reverse_controller',
        executable='agv_high_precision_reverse_controller_node_exe',
        name='agv_high_precision_reverse_controller',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('~/input/odometry', LaunchConfiguration('input_odom')),
            ('~/input/rear_warning_level',
             LaunchConfiguration('input_rear_warning_level')),
            ('~/output/trajectory', LaunchConfiguration('output_trajectory')),
            ('~/output/path_markers', '/planning/parking/path_markers'),
            ('~/output/goal', '/planning/parking/goal'),
            ('~/output/control_cmd', LaunchConfiguration('output_control_cmd')),
            ('~/output/gear_cmd', LaunchConfiguration('output_gear_cmd')),
            ('~/output/turn_indicators_cmd',
             LaunchConfiguration('output_turn_indicators_cmd')),
            ('~/output/hazard_lights_cmd',
             LaunchConfiguration('output_hazard_lights_cmd')),
            ('~/output/state', '/control/reverse/state'),
            ('~/debug/tracking', '/control/reverse/debug/tracking'),
        ],
    )
    return LaunchDescription(arguments + [node])
