cmake_minimum_required(VERSION 3.14)
project(reverse_parking_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Create library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/reverse_parking_controller_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "reverse_parking_controller::ReverseParkingControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
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
