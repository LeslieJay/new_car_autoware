cmake_minimum_required(VERSION 3.14)
project(byd_vehicle_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrailerConfiguration.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_auto_package()
