cmake_minimum_required(VERSION 3.8)
project(didrive_rear_pedestrian_warning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找所有依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tier4_perception_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 2. 添加生成可执行文件
add_executable(rear_pedestrian_warning_node src/rear_pedestrian_warning_node.cpp)

# 3. 指定头文件包含路径
target_include_directories(rear_pedestrian_warning_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# 4. 关键修正：传统的 C++ 库（如 rclcpp, cv_bridge, OpenCV）依然可以用这种方式链接
target_link_libraries(rear_pedestrian_warning_node
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  message_filters::message_filters
  ${OpenCV_LIBS}
)

# 5. 关键修正：所有 ROS 2 消息包（_msgs），必须使用 ament 官方推荐的宏来注入依赖
# 这会自动帮你处理好头文件路径和复杂的库链接
ament_target_dependencies(rear_pedestrian_warning_node
  sensor_msgs
  std_msgs
  tier4_perception_msgs
  autoware_perception_msgs
)

# 6. 设置安装路径
install(TARGETS rear_pedestrian_warning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()