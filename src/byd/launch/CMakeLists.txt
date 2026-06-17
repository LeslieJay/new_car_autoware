cmake_minimum_required(VERSION 3.8)
project(byd_launch)

find_package(ament_cmake REQUIRED)

install(FILES
  bringup.launch.py
  reverse_parking.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/setup_external_control_mode.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
