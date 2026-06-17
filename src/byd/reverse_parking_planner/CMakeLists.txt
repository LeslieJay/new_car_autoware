cmake_minimum_required(VERSION 3.14)
project(reverse_parking_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGoalPose.srv"
  DEPENDENCIES geometry_msgs
)

# Create library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/reverse_parking_planner_node.cpp
  src/reeds_shepp.cpp
)

# Link generated service interfaces to the node library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "reverse_parking_planner::ReverseParkingPlannerNode"
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
