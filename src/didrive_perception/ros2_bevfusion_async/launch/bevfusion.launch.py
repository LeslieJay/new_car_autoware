import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():
    """Launch the BEVFusion Node as a composable component."""
    
    # 声明启动参数
    launch_args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                FindPackageShare('didrive_ros2_bevfusion_async').find('didrive_ros2_bevfusion_async'),
                'config',
                'bevfusion_camera_lidar.param.yaml'
            ),
            description='Path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
        'model_path',
        default_value=[
            FindPackageShare('didrive_ros2_bevfusion_async'),
            '/models/resnet50int8'  # 默认模型路径
        ],
        description='Path to the BEVFusion model directory'
        ),
    
        # 新增：precision 参数
        DeclareLaunchArgument(
            'precision',
            default_value='int8',
            description='Inference precision: int8, fp16, fp32'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/rslidar_points',
            description='激光雷达点云输入话题'
        ),
        DeclareLaunchArgument(
            'camera0_topic',
            default_value='/cam0/image_rect',
            description='相机0图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera1_topic',
            default_value='/cam1/image_rect',
            description='相机1图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera2_topic',
            default_value='/cam2/image_rect',
            description='相机2图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera3_topic',
            default_value='/cam3/image_rect',
            description='相机3图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera4_topic',
            default_value='/cam4/image_rect',
            description='相机4图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera5_topic',
            default_value='/cam5/image_rect',
            description='相机5图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera0_info_topic',
            default_value='/cam0/camera/camera_info',
            description='相机0内参输入话题（sensor_msgs/msg/CameraInfo）'
        ),
        DeclareLaunchArgument(
            'camera1_info_topic',
            default_value='/cam1/camera/camera_info',
            description='相机1内参输入话题'
        ),
        DeclareLaunchArgument(
            'camera2_info_topic',
            default_value='/cam2/camera/camera_info',
            description='相机2内参输入话题'
        ),
        DeclareLaunchArgument(
            'camera3_info_topic',
            default_value='/sensing/camera/camera3/image_rect_color/camera_info',
            description='相机3内参输入话题'
        ),
        DeclareLaunchArgument(
            'camera4_info_topic',
            default_value='/sensing/camera/camera4/image_rect_color/camera_info',
            description='相机4内参输入话题'
        ),
        DeclareLaunchArgument(
            'camera5_info_topic',
            default_value='/sensing/camera/camera5/image_rect_color/camera_info',
            description='相机5内参输入话题'
        ),
        DeclareLaunchArgument(
            'detection_topic',
            default_value='/perception/object_recognition/detection3d',
            description='3D检测结果输出话题'
        ),
        # 新增：控制是否使用外部容器
        DeclareLaunchArgument(
            'use_pointcloud_container',
            default_value='false',
            description='If true, load node into existing container, else create new container'
        ),
        # 新增：外部容器名称
        DeclareLaunchArgument(
            'pointcloud_container_name',
            default_value='pointcloud_container',
            description='Name of the existing container to load into'
        ),
    ]

    # 定义BEVFusion组件节点
    bevfusion_node = ComposableNode(
        name='bevfusion_node_async',
        #namespace='/perception/bevfusion',
        package='didrive_ros2_bevfusion_async',
        plugin='didrive::ros2_bevfusion_async::NVBEVFusionNodeAsync',
        parameters=[
            LaunchConfiguration('params_file'),  # 从指定的YAML文件加载参数
            {
                'model_path': LaunchConfiguration('model_path'),
                'precision': LaunchConfiguration('precision'),
            }
        ],
        # remappings=[
        #     ('/input/pointcloud', LaunchConfiguration('lidar_topic')),
        #     ('/input/image0', LaunchConfiguration('camera0_topic')),
        #     ('/input/image1', LaunchConfiguration('camera1_topic')),
        #     ('/input/image2', LaunchConfiguration('camera2_topic')),
        #     ('/input/image3', LaunchConfiguration('camera3_topic')),
        #     ('/input/image4', LaunchConfiguration('camera4_topic')),
        #     ('/input/image5', LaunchConfiguration('camera5_topic')),
        #     ('/input/camera_info0', LaunchConfiguration('camera0_info_topic')),
        #     ('/input/camera_info1', LaunchConfiguration('camera1_info_topic')),
        #     ('/input/camera_info2', LaunchConfiguration('camera2_info_topic')),
        #     ('/input/camera_info3', LaunchConfiguration('camera3_info_topic')),
        #     ('/input/camera_info4', LaunchConfiguration('camera4_info_topic')),
        #     ('/input/camera_info5', LaunchConfiguration('camera5_info_topic')),
        #     ('/output/objects', LaunchConfiguration('detection_topic'))
        # ],

        remappings=[
            ('/input/pointcloud', LaunchConfiguration('lidar_topic')),
            
            # 严格根据头文件里的注释顺序来对号入座：
            ('/input/image0', LaunchConfiguration('camera0_topic')), # 索引0 -> 前视 (/cam0/image_rect)
            ('/input/image1', LaunchConfiguration('camera2_topic')), # 索引1 -> 左前 (/cam2/image_rect) 🌟注意这里
            ('/input/image2', LaunchConfiguration('camera3_topic')), # 索引2 -> 左后 (假话题，刷黑)
            ('/input/image3', LaunchConfiguration('camera1_topic')), # 索引3 -> 右前 (/cam1/image_rect) 🌟注意这里
            ('/input/image4', LaunchConfiguration('camera4_topic')), # 索引4 -> 右后 (假话题，刷黑)
            ('/input/image5', LaunchConfiguration('camera5_topic')), # 索引5 -> 后视 (假话题，刷黑)
            
            # 内参映射保持同步对应的顺序
            ('/input/camera_info0', LaunchConfiguration('camera0_info_topic')),
            ('/input/camera_info1', LaunchConfiguration('camera2_info_topic')), # 🌟同步调换
            ('/input/camera_info2', LaunchConfiguration('camera3_info_topic')),
            ('/input/camera_info3', LaunchConfiguration('camera1_info_topic')), # 🌟同步调换
            ('/input/camera_info4', LaunchConfiguration('camera4_info_topic')),
            ('/input/camera_info5', LaunchConfiguration('camera5_info_topic')),
            ('/output/objects', LaunchConfiguration('detection_topic'))
        ],

        #extra_arguments=[
        #    {'use_intra_process_comms': False},
        #]
    )

    # 创建组件容器
    container = ComposableNodeContainer(
        name=f'bevfusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 情况1：使用外部容器 → 加载到指定容器
    external_loader = LoadComposableNodes(
        composable_node_descriptions=[bevfusion_node],
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    # 情况2：使用本地容器 → 加载到本地 container
    internal_loader = LoadComposableNodes(
        composable_node_descriptions=[bevfusion_node],
        target_container=container,  # 注意：这里直接传 container 对象
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return launch.LaunchDescription(launch_args + [container, external_loader, internal_loader])
    