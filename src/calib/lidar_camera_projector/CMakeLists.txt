cmake_minimum_required(VERSION 3.14)
project(lidar_camera_projector)
#set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
# 如果使用 Clang，忽略非标准 include 路径警告
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

if(NOT AMENT_CMAKE_AUTO_FOUND)
  find_package(ament_cmake_auto REQUIRED)
endif()

# 必须显式 find_package rclcpp_components
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# ----------------------------------------
# Eigen3 依赖
# ----------------------------------------
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    highgui
    calib3d
)

# PCL
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/lidar_camera_proj_node.cpp
)

# ----------------------------------------
# 链接依赖库
# ----------------------------------------
target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# ----------------------------------------
# 注册为组件节点（rclcpp component）
# ----------------------------------------
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "LidarCameraProjNode"
  EXECUTABLE lidar_camera_proj_node
)

# ----------------------------------------
# 自动处理 ament 包的安装
# ----------------------------------------
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
