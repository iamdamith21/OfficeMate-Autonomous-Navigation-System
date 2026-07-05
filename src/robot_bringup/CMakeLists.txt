cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make the helper scripts executable when installed
install(
  PROGRAMS scripts/wheel_joint_pub.py scripts/reset_lidar.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
