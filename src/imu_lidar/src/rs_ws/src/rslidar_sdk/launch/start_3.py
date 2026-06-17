from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 rviz 配置（可选）
    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'

    # 三个雷达的配置文件路径（请根据你的实际路径修改！）
    config_top    = '/home/nvidia/autoware/src/imu_lidar/src/rs_ws/src/rslidar_sdk/config/config_top.yaml'
    config_left   = '/home/nvidia/autoware/src/imu_lidar/src/rs_ws/src/rslidar_sdk/config/config_left.yaml'
    config_right  = '/home/nvidia/autoware/src/imu_lidar/src/rs_ws/src/rslidar_sdk/config/config_right.yaml'

    return LaunchDescription([
        # Top LiDAR
        Node(
            namespace='rslidar_top',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            # name='rslidar_sdk_node',  # 可选：显式命名
            output='screen',
            parameters=[{'config_path': config_top}]
        ),
        # Left LiDAR
        Node(
            namespace='rslidar_left',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            # name='rslidar_sdk_node',
            output='screen',
            parameters=[{'config_path': config_left}]
        ),
        # Right LiDAR
        Node(
            namespace='rslidar_right',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            # name='rslidar_sdk_node',
            output='screen',
            parameters=[{'config_path': config_right}]
        ),

        # 可选：启动 RViz（取消注释即可）
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen'
        # )
    ])