cmake_minimum_required(VERSION 3.8)
project(mapping_bot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/mcu_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
