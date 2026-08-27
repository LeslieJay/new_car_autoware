cmake_minimum_required(VERSION 3.14)
project(byd_vehicle_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActiveEvent.msg"
  "msg/EventTrigger.msg"
  "msg/RecorderStatus.msg"
  "msg/TrailerConfiguration.msg"
  "srv/TriggerEvent.srv"
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
)

ament_auto_package()
