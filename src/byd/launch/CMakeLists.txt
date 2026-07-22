cmake_minimum_required(VERSION 3.8)
project(byd_launch)

find_package(ament_cmake REQUIRED)

install(FILES
  bringup.launch.py
  parallel_bringup.launch.py
  drivers.launch.py
  applications.launch.py
  reverse_parking.launch.py
  pedestrian_safety_stop.launch.py
  launch_utils.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/setup_external_control_mode.sh
  scripts/wait_for_ros_ready.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(PROGRAMS
  scripts/check_autonomous_transition.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_check_autonomous_transition
    test/test_check_autonomous_transition.py
  )
  ament_add_pytest_test(test_pedestrian_safety_stop
    test/test_pedestrian_safety_stop.py
  )
endif()

ament_package()
