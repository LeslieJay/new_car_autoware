cmake_minimum_required(VERSION 3.8)
project(vda5050_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/ActionParameters.msg"
  "msg/Value.msg"
  "msg/Nodes.msg"
  "msg/Edges.msg"
  "msg/NodeStates.msg"
  "msg/EdgeStates.msg"
  "msg/ActionStates.msg"
  "msg/BatteryState.msg"
  "msg/Actions.msg"
  "msg/TypeSpecification.msg"
  "msg/PhysicalParameters.msg"
  "msg/AGVPosition.msg"
  "msg/Velocity.msg"
  "msg/NodePosition.msg"
  "msg/AGVOrder.msg"
  "msg/AGVState.msg"
  "msg/AGVInstantActions.msg"
  "msg/AGVFactsheet.msg"
  "msg/AGVVisualization.msg"
  "msg/AGVConnection.msg"
  "msg/ControlPoints.msg"
  "msg/Trajectory.msg"
  "msg/Error.msg"
  "msg/ErrorReference.msg"
  "msg/SafetyState.msg"
  "action/AGVConnection.action"

)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
