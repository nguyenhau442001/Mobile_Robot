cmake_minimum_required(VERSION 3.10)
project(mobile_robot_digital_twin)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/digital_twin_sync_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
