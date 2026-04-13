cmake_minimum_required(VERSION 3.8)
project(substation_nav_demo)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/auto_goal_sender.py
  scripts/goal_marker_publisher.py
  scripts/spawn_obstacle_ahead.py
  scripts/multi_ugv_patrol_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
