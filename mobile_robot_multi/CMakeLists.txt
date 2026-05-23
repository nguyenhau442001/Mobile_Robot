cmake_minimum_required(VERSION 3.5)
project(mobile_robot_multi)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dispatch_goals.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
