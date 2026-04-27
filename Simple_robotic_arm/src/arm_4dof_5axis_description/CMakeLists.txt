cmake_minimum_required(VERSION 3.8)
project(arm_4dof_5axis_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
