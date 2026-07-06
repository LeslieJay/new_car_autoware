from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 1. 定义两个不同工作空间的二进制文件绝对路径
    ws1_path = "/media/f/nvme_storage/can_ws"
    ws2_path = "/media/f/nvme_storage/usb_can_ws"
    
    # 确认路径拼接正确
    rtk_bin_path = os.path.join(ws1_path, "install", "lib", "can_driver", "can_rtk_node")
    usb_bin_path = os.path.join(ws2_path, "install", "lib", "can_driver", "can_node")

    # 2. 定义节点
    # 技巧：executable 随便填一个名字（用于日志显示），真正的路径放在 prefix 里
    rtk_node = Node(
        package='can_driver',
        executable='can_rtk_node',  # 这里只填名字，用于显示
        name='rtk_node',
        output='screen',
        prefix=[rtk_bin_path]       # 【关键】在这里放入绝对路径
    )

    usb_node = Node(
        package='can_driver',
        executable='can_node',      # 这里只填名字
        name='usb_node',
        output='screen',
        prefix=[usb_bin_path]       # 【关键】在这里放入绝对路径
    )

    return LaunchDescription([rtk_node, usb_node])