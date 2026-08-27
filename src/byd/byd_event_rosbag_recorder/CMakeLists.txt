cmake_minimum_required(VERSION 3.14)
project(byd_event_rosbag_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(byd_vehicle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)

add_library(event_recorder_core src/event_recorder_core.cpp)
target_include_directories(event_recorder_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(event_recorder_core diagnostic_msgs)

add_executable(event_rosbag_recorder_node src/event_rosbag_recorder_node.cpp)
target_link_libraries(event_rosbag_recorder_node event_recorder_core)
ament_target_dependencies(event_rosbag_recorder_node
  byd_vehicle_msgs diagnostic_msgs rclcpp rosbag2_cpp rosbag2_storage std_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS event_recorder_core EXPORT export_event_recorder_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS event_rosbag_recorder_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_event_recorder_core test/test_event_recorder_core.cpp)
  target_link_libraries(test_event_recorder_core event_recorder_core)
endif()

ament_export_targets(export_event_recorder_core HAS_LIBRARY_TARGET)
ament_package()
