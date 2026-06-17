cmake_minimum_required(VERSION 3.8)
project(lidar_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖 (保持不变)
find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED) 
find_package(pcl_conversions REQUIRED)
find_package(autoware_point_types REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 自动添加库 (修改点：列出所有源文件)
# 这会将两个 .cpp 编译进同一个共享库中
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lidar_transform.cpp
  src/side_lidar_cropbox.cpp
)

# 3. 指定依赖 (保持不变，确保包含所有用到的包)
ament_target_dependencies(${PROJECT_NAME}
  SYSTEM
  rclcpp 
  rclcpp_components
  sensor_msgs
  pcl_conversions
  PCL  
  autoware_point_types
  Eigen3
)
# 4. 注册组件 
# 节点 1: 原始转换节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lidar_transform::LidarTransformNode"
  EXECUTABLE lidar_transform_node
)

# 节点 2: 侧向雷达裁剪节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lidar_transform::SideLidarCropBoxNode"
  EXECUTABLE side_lidar_cropbox_node
)


# 5. 自动化打包与安装
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)