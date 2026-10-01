cmake_minimum_required(VERSION 3.8)
project(a1_gnss_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/gnss_odom_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
