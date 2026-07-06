#!/usr/bin/env python3
"""启动 CAN 驱动、RTK CAN 驱动、三路雷达、agv_to_rcs 及倒车泊车。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rslidar_launch = os.path.join(
        get_package_share_directory('rslidar_sdk'),
        'launch',
        'start_3.py',
    )

    agv_to_rcs_launch = os.path.join(
        get_package_share_directory('agv_to_rcs'),
        'launch',
        'agv_to_rcs.launch.py',
    )

    reverse_parking_launch = os.path.join(
        get_package_share_directory('byd_launch'),
        'launch',
        'reverse_parking.launch.py',
    )

    can_node = Node(
        package='can_driver',
        executable='can_node',
        output='screen',
    )

    can_rtk_node = Node(
        package='can_rtk_driver',
        executable='can_rtk_node',
        output='screen',
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rslidar_launch),
    )

    agv_to_rcs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(agv_to_rcs_launch),
    )

    reverse_parking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(reverse_parking_launch),
    )

    return LaunchDescription([
        can_node,
        can_rtk_node,
        lidar_launch,
        agv_to_rcs,
        reverse_parking,
    ])
