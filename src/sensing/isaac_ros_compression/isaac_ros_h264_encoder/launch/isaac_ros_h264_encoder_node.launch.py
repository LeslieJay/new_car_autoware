import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import uuid


def generate_launch_description():
    """Launch the H.264 Encoder Node."""
    launch_args = [
        DeclareLaunchArgument(
            'input_topic',
            default_value='/sensing/camera/camera0/image_raw',
            description='输入的RGB8格式图像话题（字符串，内部为sensor_msgs/msg/Image话题）'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/sensing/camera/camera0/image_raw/compressed',
            description='输出的H.264压缩图像话题（字符串，内部为sensor_msgs/msg/CompressedImage话题）'
        ),
        DeclareLaunchArgument(
            'input_height',
            default_value='1080',
            description='Height of the original image'),
        DeclareLaunchArgument(
            'input_width',
            default_value='1920',
            description='Width of the original image'),
        DeclareLaunchArgument(
            'config',
            default_value='pframe_cqp',
            description='Config of encoder'),
    ]

    # Encoder parameters
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    input_height = LaunchConfiguration('input_height')
    input_width = LaunchConfiguration('input_width')
    config = LaunchConfiguration('config')
    name_uuid = str(uuid.uuid4())[:8]

    h264_encoder_node = ComposableNode(
        name=f'h264_encoder_node_{name_uuid}',
        namespace=f'/h264_encoder_node/id_{name_uuid}',
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        parameters=[{
                'input_height': input_height,
                'input_width': input_width,
                'config': config,
                'hw_preset_type': 0,
                "qp": 20,
                "profile": 0,
                "iframe_interval": 5,
                # 'input_image_encoding': 'rgb8',
                # 'block_memory_pool_size': 6220800 * 2,
        }],
        remappings=[
            ('image_raw', input_topic),
            ('image_compressed', output_topic)
        ],
        extra_arguments=[
            {'use_intra_process_comms': False},
            {'extensions': ['gxf/std', 'gxf/cuda', 'gxf/isaac_media', 'gxf/isaac_tensorops', 'gxf/serialization']}
        ]
    )

    container = ComposableNodeContainer(
        name=f'h264_encoder_container_{name_uuid}',
        namespace=f'/h264_encoder_container/id_{name_uuid}',
        # namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[h264_encoder_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return (launch.LaunchDescription(launch_args + [container]))
