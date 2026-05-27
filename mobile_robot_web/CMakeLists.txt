cmake_minimum_required(VERSION 3.10)
project(mobile_robot_web)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
