cmake_minimum_required(VERSION 3.8)
project(multi_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(COMPILER_WARNINGS)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找 ROS 2 核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 2. 查找 Autoware 消息依赖
find_package(autoware_vehicle_msgs REQUIRED)

# 3. 查找常规第三方库
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# 4. 定义可执行文件
add_executable(multi_recorder_node src/multi_recorder_node.cpp)

# 5. ⭐ 核心修复：使用这个现代 ROS 2 宏自动注入所有 ROS 依赖的头文件和库路径
ament_target_dependencies(multi_recorder_node
  rclcpp
  sensor_msgs
  cv_bridge
  autoware_vehicle_msgs
)

# 6. 额外注入 OpenCV 和 GStreamer 的头文件路径与库链接
target_include_directories(multi_recorder_node PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(multi_recorder_node
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

# 7. 安装路径配置
install(TARGETS
  multi_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()