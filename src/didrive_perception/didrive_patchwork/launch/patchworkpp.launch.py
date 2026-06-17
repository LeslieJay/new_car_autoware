from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import LoadComposableNodes

# This configuration parameters are not exposed thorugh the launch system, meaning you can't modify
# those throw the ros launch CLI. If you need to change these values, you could write your own
# launch file and modify the 'parameters=' block from the Node class.
class config:
    # TBU. Examples are as follows:
    max_range: float = 80.0
    # deskew: bool = False


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)
    

    # 声明启动参数
    launch_args = [
        DeclareLaunchArgument(
            'input_pointcloud',
            default_value='/rslidar_points',
            description='input origin lidar topic'
        ),
        DeclareLaunchArgument(
            'output_ground',
            default_value='/output/ground',
            description='ground pointcloud'
        ),
        DeclareLaunchArgument(
            'output_nonground',
            default_value="/output/noground",
            description='non ground pointcloud'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value="base_link",
            description='base frame id'
        ),
        DeclareLaunchArgument(
            'mask_frame',
            default_value="camera_front_120/camera_optical_link",
            description='base frame id'
        ),
        DeclareLaunchArgument(
            'use_pointcloud_container',
            default_value='false',
            description='If true, load node into existing container, else create new container'
        ),
        DeclareLaunchArgument(
            'pointcloud_container_name',
            default_value='pointcloud_container',
            description='Name of the existing container to load into'
        ),
        DeclareLaunchArgument(
            'use_mask',
            default_value='true',
            description='If true, use image mask to filter ground points'
        ),
    ]


    # Patchwork++ node
    patchworkpp_node = ComposableNode(
        package="didrive_patchwork",
        plugin="didrive::patchworkpp_ros::PatchworkppSegNode", 
        name="patchworkpp_seg_node",
        #prefix=['gdbserver', ' :1234'],
        remappings=[
            ("input_pointcloud", LaunchConfiguration("input_pointcloud")),
            ("output_ground", LaunchConfiguration("output_ground")),
            ("output_nonground", LaunchConfiguration("output_nonground")),
            ("input_image", '/perception/obstacle_segmentation/mask')
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": LaunchConfiguration("base_frame"),
                "mask_frame": LaunchConfiguration("mask_frame"),
                # Patchwork++ configuration
                "sensor_height": 2.30, #0.25
                "num_iter": 3,  # Number of iterations for ground plane estimation using PCA.
                "num_lpr": 20,  # Maximum number of points to be selected as lowest points representative.
                "num_min_pts": 10,  # Minimum number of points to be estimated as ground plane in each patch.
                "th_seeds": 0.2,
                # threshold for lowest point representatives using in initial seeds selection of ground points.
                "th_dist": 0.1,  # threshold for thickness of ground.
                "th_seeds_v": 0.25,
                # threshold for lowest point representatives using in initial seeds selection of vertical structural points.
                "th_dist_v": 0.1,  # threshold for thickness of vertical structure.
                "max_range": 80.0,  # max_range of ground estimation area
                "min_range": 1.8,  # min_range of ground estimation area
                "uprightness_thr": 0.701,
                # threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE.
                "verbose": False,  # display verbose info
                "use_mask": LaunchConfiguration("use_mask"),
                "max_time_diff": 0.5,
                "camera_intrinsic": [266.93, 0., 445.93,
                                     0.0, 396.97, 258.19,
                                     0.0,  0.0,    1.0
                                ],
            }
        ],
    )
    
    container = ComposableNodeContainer(
        name=f'patchwork_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    # 情况1：使用外部容器 → 加载到指定容器
    external_loader = LoadComposableNodes(
        composable_node_descriptions=[patchworkpp_node],
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    # 情况2：使用本地容器 → 加载到本地 container
    internal_loader = LoadComposableNodes(
        composable_node_descriptions=[patchworkpp_node],
        target_container=container,  # 注意：这里直接传 container 对象
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    return LaunchDescription(launch_args + [container, external_loader, internal_loader])

