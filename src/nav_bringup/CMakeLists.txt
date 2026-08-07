cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf rviz maps meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
