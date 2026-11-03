cmake_minimum_required(VERSION 3.8)
project(a1_lidar_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config calibration
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
