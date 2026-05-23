cmake_minimum_required(VERSION 3.5)
project(mobile_robot_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
