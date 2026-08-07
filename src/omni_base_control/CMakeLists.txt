cmake_minimum_required(VERSION 3.8)
project(omni_base_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/omni_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
