cmake_minimum_required(VERSION 3.5)
project(can_dy_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(can_rtk_node
  src/can_node.cpp
  src/can_receiver.cpp
  src/main.cpp
)

ament_target_dependencies(can_rtk_node
  rclcpp
  geometry_msgs
  sensor_msgs
  autoware_sensing_msgs
  tf2
  ament_index_cpp
)


set_target_properties(can_rtk_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN/../../lib"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS can_rtk_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
