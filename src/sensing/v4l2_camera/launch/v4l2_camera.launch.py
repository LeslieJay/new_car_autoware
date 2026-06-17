from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明所有启动参数
    declared_arguments = []
    
    # 相机列表：1表示启用该路相机（会声明参数并加载节点），0表示完全禁用
    cameras = [0, 0, 1, 0, 0, 0, 0]  # 7路相机全部启用
    
    # 基础参数
    declared_arguments.extend([
        DeclareLaunchArgument(
            'v4l2_camera_param_dir',
            default_value=PathJoinSubstitution([
                FindPackageShare('v4l2_camera'),
                'config'
            ]),
            description='v4l2相机参数文件目录'
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='',
            description='组件容器名称（为空则使用独立节点）'
        ),
        DeclareLaunchArgument(
            'use_intra_process',
            default_value='True',
            description='是否启用进程内通信'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='-1.0',
            description='发布帧率（<=0表示无限制）'
        ),
        DeclareLaunchArgument(
            'compression_quality',
            default_value='80',
            description='压缩质量（0-100，仅JPEG有效）'
        ),
        DeclareLaunchArgument(
            'compression_format',
            default_value='jpeg',
            description='压缩格式（jpeg/png）'
        ),
        DeclareLaunchArgument(
            'undistort_enabled',
            default_value='True',
            description='是否启用去畸变'
        )
    ])
    
    # 根据相机列表为需要启用的相机声明参数
    for i in range(len(cameras)):
        if cameras[i]:
            declared_arguments.extend([
                DeclareLaunchArgument(
                    f'camera{i}_enabled',
                    default_value='True',
                    description=f'是否启用第{i}路相机'
                ),
                DeclareLaunchArgument(
                    f'image{i}_topic',
                    default_value=f'/sensing/camera/camera{i}/image_rect_color',
                    description=f'第{i}路相机的图像输出话题'
                )
            ])
    
    # 配置参数变量
    container_name = LaunchConfiguration('container_name')
    use_intra_process = LaunchConfiguration('use_intra_process')
    publish_rate = LaunchConfiguration('publish_rate')
    compression_quality = LaunchConfiguration('compression_quality')
    compression_format = LaunchConfiguration('compression_format')
    undistort_enabled = LaunchConfiguration('undistort_enabled')
    param_dir = LaunchConfiguration('v4l2_camera_param_dir')
    
    # 创建容器使用条件的表达式
    container_condition_expr = PythonExpression(['"', container_name, '" != ""'])
    
    # 存储所有相机的节点配置
    camera_actions = []
    
    # 只为相机列表中启用的相机创建节点配置
    for i in range(len(cameras)):
        if cameras[i]:
            camera_enabled = LaunchConfiguration(f'camera{i}_enabled')
            image_topic = LaunchConfiguration(f'image{i}_topic')
            
            # 独立节点模式配置 - 使用UnlessCondition并传入表达式
            standalone_nodes = GroupAction([
                Node(
                    package='v4l2_camera',
                    executable='v4l2_camera_node',
                    name=f'v4l2_camera_{i}',
                    parameters=[
                        PathJoinSubstitution([
                            param_dir,
                            f'camera{i}.param.yaml'
                        ]),
                        {'undistort_enabled': undistort_enabled},
                        {'publish_rate': publish_rate},
                        {'image_topic': image_topic}
                    ]
                ),
                Node(
                    package='image_transport',
                    executable='republish',
                    name=f'image_compressor_{i}',
                    arguments=['raw', 'compressed'],
                    parameters=[
                        {'in': image_topic},
                        {'out': f'{image_topic}/compressed'},
                        {'format': compression_format},
                        {'jpeg_quality': compression_quality} if compression_format == 'jpeg' else {},
                        {'png_level': 3} if compression_format == 'png' else {}
                    ]
                )
            ], condition=UnlessCondition(container_condition_expr))
            
            # 组件容器模式配置 - 使用IfCondition并传入表达式
            composable_nodes = GroupAction([
                LoadComposableNodes(
                    target_container=container_name,
                    composable_node_descriptions=[
                        ComposableNode(
                            package='v4l2_camera',
                            plugin='v4l2_camera::V4L2Camera',
                            name=f'v4l2_camera_{i}',
                            parameters=[
                                PathJoinSubstitution([
                                    param_dir,
                                    f'camera{i}.param.yaml'
                                ]),
                                {'undistort_enabled': undistort_enabled},
                                {'publish_rate': publish_rate},
                                {'image_topic': image_topic}
                            ],
                            extra_arguments=[{'use_intra_process_comms': use_intra_process}]
                        ),
                        ComposableNode(
                            package='image_transport',
                            plugin='image_transport::RepublishNode',
                            name=f'image_compressor_{i}',
                            parameters=[
                                {'in': image_topic},
                                {'out': f'{image_topic}/compressed'},
                                {'format': compression_format},
                                {'jpeg_quality': compression_quality} if compression_format == 'jpeg' else {},
                                {'png_level': 3} if compression_format == 'png' else {}
                            ],
                            extra_arguments=[
                                  {'use_intra_process_comms': use_intra_process},
                                  {'args': ['raw', 'compressed']},
                                ]
                        )
                    ]
                )
            ], condition=IfCondition(container_condition_expr))
            
            # 添加带启用条件的相机配置
            camera_actions.append(GroupAction([
                standalone_nodes,
                composable_nodes
            ], condition=IfCondition(camera_enabled)))
    
    # 生成启动描述
    return LaunchDescription(declared_arguments + camera_actions)
    