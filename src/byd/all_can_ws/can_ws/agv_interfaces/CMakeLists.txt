cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 接口文件配置
set(msg_files
  "msg/BatteryState.msg"
  "msg/DifferentialWheel.msg"
  "msg/Odom.msg"
  "msg/Point.msg"
  "msg/Points.msg"
  "msg/Poses.msg"
  "msg/QrMsg.msg"
  "msg/TrayState.msg"
  "msg/VehicleStatus.msg"
  "msg/AutomaticSwitch.msg"
)

set(srv_files
  "srv/BatteryControl.srv"
  "srv/ScramSiginal.srv" #watch dog
)

set(action_files
  "action/AGVSend.action"
  "action/TrayControl.action"
)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  std_msgs
  action_msgs
)

# 安装配置
install(
  DIRECTORY msg srv action
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(action_msgs)

ament_package()
