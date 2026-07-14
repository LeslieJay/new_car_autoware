cmake_minimum_required(VERSION 3.8)
project(agv_to_rcs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "Using C++ standard: ${CMAKE_CXX_STANDARD}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vda5050_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(ref_slam_interface REQUIRED)
find_package(agv_interfaces REQUIRED)
find_package(reverse_parking_planner REQUIRED)


include_directories(
  include/agv_to_rcs
  ${catkin_INCLUDE_DIRS}
)

add_executable(     main src/main.cpp

                    src/agv_config.cpp

                    src/client_end_guide.cpp
                    src/client_fork_action.cpp
                    src/client_rack_detection.cpp
                    src/client_request_charging.cpp
                    src/client_send_multi_poses.cpp

                    src/control_agv_battery.cpp
                    src/control_agv_driver.cpp
                    src/control_agv_fork.cpp
                    src/control_execute_action.cpp

                    src/publishers_agv_state.cpp
                    src/mqtt_client.cpp
                    src/mqtt_message_handler.cpp
                    src/json_converter.cpp

                    src/server_obstacle.cpp

                    src/state_detecting_behaviors.cpp
                    src/state_error_behaviors.cpp
                    src/state_idle_behaviors.cpp
                    src/state_init_behaviors.cpp
                    src/state_lock_behaviors.cpp
                    src/state_running_behaviors.cpp
                    
                    src/subscriber_battery.cpp
                    src/subscriber_candata.cpp
                    src/subscriber_current_pose.cpp
                    src/subscriber_instant_action.cpp
                    src/subscriber_order.cpp
                    src/subscriber_velocity.cpp
                    
                    src/tool_black_box.cpp
                    src/tool_math.cpp
)


add_executable(     agv_localization src/tool_localization.cpp

                    src/agv_config.cpp

                    src/client_request_charging.cpp
)

add_executable(     quick_tool src/tool_quick.cpp

                    src/agv_config.cpp

                    src/client_end_guide.cpp
                    src/client_request_charging.cpp
                    src/client_fork_action.cpp
                    src/client_send_multi_poses.cpp

                    src/server_obstacle.cpp

                    src/subscriber_current_pose.cpp

                    src/tool_math.cpp
)
# 创建可执行文件
add_executable(autoware_pose_to_rcs_pose 
  src/autoware_pose_to_rcs_pose.cpp
)

add_executable(autoware_auto_server
  src/autoware_auto_server.cpp
)

# 原版服务用于轨迹验证，暂时取消了
add_executable(goal_service_server
  src/use_trajectory_server.cpp
)

ament_target_dependencies(
  autoware_auto_server
  "rclcpp"
  "reverse_parking_planner"
  "autoware_system_msgs"
  "rclcpp_action"
  "geometry_msgs"
  "ref_slam_interface"
  "nav_msgs"
  "vda5050_interfaces"
  "tf2_geometry_msgs"
)

# 添加依赖
ament_target_dependencies(autoware_pose_to_rcs_pose
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(agv_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(agv_localization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(quick_tool PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(quick_tool PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(goal_service_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(goal_service_server PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  main
  "rclcpp"
  "geometry_msgs"
  "vda5050_interfaces"
  "rclcpp_action"
  "nav2_msgs"
  "tf2_geometry_msgs"
  "ref_slam_interface"
  "agv_interfaces"
  "behaviortree_cpp"
)

ament_target_dependencies(
  goal_service_server
  "rclcpp"
  "geometry_msgs"
  "ref_slam_interface"
  "nav_msgs"
  "vda5050_interfaces"
)
ament_target_dependencies(
  agv_localization
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "tf2_ros"
  "nav_msgs"
  "vda5050_interfaces"
  "ref_slam_interface"
  "agv_interfaces"
)


ament_target_dependencies(
  quick_tool
  "rclcpp"
  "rclcpp_action"
  "vda5050_interfaces"
  "ref_slam_interface"
  "agv_interfaces"
  "nav2_msgs"
  "tf2_geometry_msgs"
)


target_link_libraries(main yaml-cpp PahoMqttCpp::paho-mqttpp3)
target_link_libraries(agv_localization yaml-cpp)
target_link_libraries(quick_tool yaml-cpp)


install(TARGETS 
  main
  agv_localization
  quick_tool
  autoware_pose_to_rcs_pose
  autoware_auto_server
  goal_service_server
  DESTINATION lib/${PROJECT_NAME})

# 安装配置文件
install(FILES src/agv_config.yaml
  DESTINATION share/${PROJECT_NAME}/configs
)
# install(FILES params/agv_config.yaml DESTINATION share/${PROJECT_NAME}/configs)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(CMAKE_BUILD_TYPE debug)#debug模式，程序不会被优化，速度非常慢
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()