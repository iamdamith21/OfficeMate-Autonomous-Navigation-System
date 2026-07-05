cmake_minimum_required(VERSION 3.8)
project(robot_mapping)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY slam map_saver room_mapper maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
