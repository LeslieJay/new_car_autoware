import launch
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    GroupAction,
    Shutdown,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import ast
import uuid

def create_nodes(context):
    # 在回调中解析参数
    input_topics_str = LaunchConfiguration('input_topics').perform(context)
    output_topics_str = LaunchConfiguration('output_topics').perform(context)
    
    # 将字符串转换为列表
    input_topics = ast.literal_eval(input_topics_str)
    output_topics = ast.literal_eval(output_topics_str)
    
    # 校验话题数量一致
    if len(input_topics) != len(output_topics):
        raise ValueError("input_topics和output_topics的数量必须一致")
    
    name_uuid = str(uuid.uuid4())[:8]
    
    # 生成多路解码器节点
    composable_nodes = []
    for i in range(len(input_topics)):
        # H.264解码器节点
        h264_decoder_node = ComposableNode(
            package='isaac_ros_h264_decoder',
            plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
            name=f'h264_decoder_node_{name_uuid}',
            namespace=f'/h264_decoder_node/id_{name_uuid}_{i}',
            parameters=[{
                'input_encoding': 'h264',
                'output_compatible_format': 'sensor_msgs/msg/Image',
            }],
            remappings=[
                ('image_compressed', input_topics[i]),
                ('image_uncompressed', output_topics[i]),
            ],
            extra_arguments=[
                {'use_intra_process_comms': False},
                {'extensions': ['gxf/std', 'gxf/cuda', 'gxf/isaac_media', 'gxf/isaac_tensorops', 'gxf/serialization']}
            ],
        )
        composable_nodes.append(h264_decoder_node)

    container = ComposableNodeContainer(
        name=f'h264_decoder_container_{name_uuid}',
        namespace=f'/h264_decoder_container/id_{name_uuid}',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
        arguments=[
            '--ros-args', '--log-level', 'info',
            '-p', 'gxf_scheduler:=GreedyScheduler',  # 显式指定GXF调度器
            '-p', 'gxf_scheduler.max_threads:=2',  # 根据节点数量调整
            '-p', 'nvdec_enabled:=true',
            '-p', 'gxf_scheduler.priority:=60',
        ],
    )
    return [container]

def generate_launch_description():
    """Launch the H.264 Decoder Node with configurable parameters."""
    # 声明可配置参数
    launch_args = [
        DeclareLaunchArgument(
            'input_topics',
            default_value="['/sensing/camera/camera0/image_rect_color/compressed', "
                          "'/sensing/camera/camera1/image_rect_color/compressed']",
            description='输入的H.264压缩图像话题列表（用逗号分隔的字符串，内部为sensor_msgs/msg/CompressedImage话题）'
        ),
        DeclareLaunchArgument(
            'output_topics',
            default_value="['/sensing/camera/camera0/image_rect_color/uncompressed', "
                          "'/sensing/camera/camera1/image_rect_color/uncompressed']",
            description='输出的解压缩图像话题列表（用逗号分隔的字符串，内部为sensor_msgs/msg/Image话题）'
        ),
        DeclareLaunchArgument(
            'rosbag_path',
            default_value='',
            description='Path of the rosbag'),
            
        DeclareLaunchArgument(
                    'use_clock',
                    default_value='true',
                    description='是否启用 --clock'
                ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='播放速率，对应 -r 参数'
        ),
    ]

    rosbag_play = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                '--clock',  # 通常都建议启用
                '-r', LaunchConfiguration('playback_rate'),
                LaunchConfiguration('rosbag_path')
            ],
            output='screen',
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration('rosbag_path'), "' != ''"])
            ),
            name='rosbag_play_process'
        )

    # 解码节点容器：仅当rosbag_path非空时启动（与播包同步）
    decoder_group = GroupAction(
        actions=[OpaqueFunction(function=create_nodes)],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('rosbag_path'), "' != ''"])
        ),
    )

    # 定义事件处理器：播包完成后退出
    shutdown_on_rosbag_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_play,
            on_exit=[Shutdown(reason="rosbag播放完成，自动退出解码节点")]
        )
    )

    return launch.LaunchDescription(launch_args + [rosbag_play, decoder_group, shutdown_on_rosbag_exit])
