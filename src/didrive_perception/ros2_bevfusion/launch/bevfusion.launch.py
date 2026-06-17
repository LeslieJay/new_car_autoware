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
            'model_path',
            default_value=[
            FindPackageShare('didrive_ros2_bevfusion'),
            '/models/resnet50int8'  # 默认模型路径
            ],
            description='模型文件路径'
        ),
        DeclareLaunchArgument(
            'precision',
            default_value='fp16',
            description='推理精度模式 (fp16/int8)'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='0.12',
            description='检测置信度阈值'
        ),
        DeclareLaunchArgument(
            'enable_timer',
            default_value='true',
            description='是否启用性能计时器'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/sensing/lidar/rslidar_32/rslidar_points',
            description='激光雷达点云输入话题'
        ),
        DeclareLaunchArgument(
            'camera0_topic',
            default_value='/sensing/camera/camera0/image_rect_color',
            description='相机0图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera1_topic',
            default_value='/sensing/camera/camera1/image_rect_color',
            description='相机1图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera2_topic',
            default_value='/sensing/camera/camera2/image_rect_color',
            description='相机2图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera3_topic',
            default_value='/sensing/camera/camera3/image_rect_color',
            description='相机3图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera4_topic',
            default_value='/sensing/camera/camera4/image_rect_color',
            description='相机4图像输入话题'
        ),
        DeclareLaunchArgument(
            'camera5_topic',
            default_value='/sensing/camera/camera5/image_rect_color',
            description='相机5图像输入话题'
        ),
        DeclareLaunchArgument(
            'detection_topic',
            default_value='/perception/object_recognition/detection/lidar_dnn/objects',
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

    # 配置参数
    model_path = LaunchConfiguration('model_path')
    precision = LaunchConfiguration('precision')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    enable_timer = LaunchConfiguration('enable_timer')
    
    # 话题重映射配置
    lidar_topic = LaunchConfiguration('lidar_topic')
    camera0_topic = LaunchConfiguration('camera0_topic')
    camera1_topic = LaunchConfiguration('camera1_topic')
    camera2_topic = LaunchConfiguration('camera2_topic')
    camera3_topic = LaunchConfiguration('camera3_topic')
    camera4_topic = LaunchConfiguration('camera4_topic')
    camera5_topic = LaunchConfiguration('camera5_topic')
    detection_topic = LaunchConfiguration('detection_topic')

    # 定义BEVFusion组件节点
    bevfusion_node = ComposableNode(
        name='bevfusion_node',
        #namespace='/perception/bevfusion',
        package='didrive_ros2_bevfusion',
        plugin='didrive::ros2bevfusion::NVBEVFusionNode',
        parameters=[{
            'model_path': model_path,
            'precision': precision,
            'confidence_threshold': confidence_threshold,
            'enable_timer': enable_timer,
            'max_time_diff': 0.5,
        }],
        remappings=[
            ('/sensing/lidar/rslidar_32/rslidar_points', lidar_topic),
            ('/sensing/camera/camera0/image_rect_color', camera0_topic),
            ('/sensing/camera/camera1/image_rect_color', camera1_topic),
            ('/sensing/camera/camera2/image_rect_color', camera2_topic),
            ('/sensing/camera/camera3/image_rect_color', camera3_topic),
            ('/sensing/camera/camera4/image_rect_color', camera4_topic),
            ('/sensing/camera/camera5/image_rect_color', camera5_topic),
            ('/perception/object_recognition/detection/lidar_dnn/objects', detection_topic)
        ],
    )

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
    