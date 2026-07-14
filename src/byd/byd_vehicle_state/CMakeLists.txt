cmake_minimum_required(VERSION 3.14)
project(byd_vehicle_state)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/vehicle_state_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "byd_vehicle_state::VehicleStateNode"
  EXECUTABLE vehicle_state_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
