'''
Author: du.xiaoying1
Date: 2025-08-08 15:06:56
LastEditors: dxy
LastEditTime: 2025-08-13 11:12:38
FilePath: /qr_agv_0627_r/src/can_driver/launch/can_launch.py
Description: 

Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved. 
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的路径，并声明参数
    package_name = 'can_rtk_driver'
    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'can_params.yaml'
    )

    # 声明启动参数：配置文件路径
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the YAML parameter file.'
    )

    # 检查 YAML 文件是否存在
    def check_yaml_exists(context: LaunchContext):
        params_file_path = context.launch_configurations['params_file']
        if not os.path.exists(params_file_path):
            return [LogInfo(msg=f"ERROR: Parameters file does not exist: {params_file_path}")]
        return []

    # 定义 CAN 节点
    can_rtk_node = Node(
        package=package_name,
        executable='can_rtk_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=check_yaml_exists),
        can_rtk_node
    ])