cmake_minimum_required(VERSION 3.14)
project(byd_system_event_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(byd_vehicle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

add_library(system_event_detector src/system_event_detector.cpp)
target_include_directories(system_event_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(system_event_monitor_node src/system_event_monitor_node.cpp)
target_link_libraries(system_event_monitor_node system_event_detector)
ament_target_dependencies(system_event_monitor_node
  autoware_internal_planning_msgs
  autoware_planning_msgs
  autoware_system_msgs
  autoware_vehicle_msgs
  byd_vehicle_msgs
  diagnostic_msgs
  nav_msgs
  rclcpp
  tier4_planning_msgs)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS system_event_detector EXPORT export_system_event_detector
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS system_event_monitor_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_system_event_detector test/test_system_event_detector.cpp)
  target_link_libraries(test_system_event_detector system_event_detector)
endif()

ament_export_targets(export_system_event_detector HAS_LIBRARY_TARGET)
ament_package()
