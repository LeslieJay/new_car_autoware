from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包共享目录路径
    pkg_share_dir = get_package_share_directory('didrive_ufld_ros')
    
    # 参数文件路径
    params_file = os.path.join(pkg_share_dir, 'config', 'params.yaml')
    launch_args = [
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/sensing/camera/camera0/image_rect_color',
            description='input camera topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/lane_detection/points',
            description='output points topic'
        ),
        DeclareLaunchArgument(
        'model_path',
        default_value=[
            FindPackageShare('didrive_ufld_ros'),
            '/model/tmp_own_fp16.trt'  # 默认模型路径
        ],
        description='Path to the trt model directory'
        ),
        DeclareLaunchArgument(
        'save_vis_image',
        default_value='false', 
        description='save vis_image'
        ),
    ]
    # 创建节点
    ufld_node = Node(
        package='didrive_ufld_ros',
        executable='ufld_ros_node',
        name='ufld_detector',
        output='screen',
        parameters=[params_file, 
                    {'engine_path': LaunchConfiguration('model_path'),
                     'image_topic': LaunchConfiguration('camera_topic'),
                     'pointcloud_topic': LaunchConfiguration('output_topic'),
                     'save_vis_image': LaunchConfiguration('save_vis_image')}],
        remappings=[]
    )
    
    return LaunchDescription(
        launch_args + [ufld_node]
    )