cmake_minimum_required(VERSION 3.8)
project(byd_auto_engage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

include_directories(include)

add_executable(auto_engage_node src/auto_engage_node.cpp)
ament_target_dependencies(auto_engage_node
  rclcpp
  geometry_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS auto_engage_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
