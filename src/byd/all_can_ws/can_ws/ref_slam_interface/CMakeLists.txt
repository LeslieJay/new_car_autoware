cmake_minimum_required(VERSION 3.5)
project(ref_slam_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # for custom interfaces
find_package(Eigen3 REQUIRED) # for Refelctor.hpp
find_package(nav_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryState.msg"
  "msg/ObstacleState.msg"
  "msg/ObstacleChannels.msg"
  "msg/Racks.msg"
  "msg/VehicleStatus.msg"
  "msg/IoData.msg"
  "msg/ErrData.msg"
  "msg/HardwareData.msg"
  
  "srv/SaveMap.srv"
  "srv/GlbLocReq.srv"
  "srv/BatteryControl.srv"
  "srv/ObstacleDetection.srv"
  "srv/UseTrajectory.srv"
  "srv/EndGuide.srv"
  "srv/RackDetection.srv"
  "srv/ScramSignal.srv"

  "action/AutowareAuto.action"
  "action/CtrlFork.action"
  "action/MicroMotionControl.action"
  
  DEPENDENCIES nav_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
