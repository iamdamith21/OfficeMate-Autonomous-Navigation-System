cmake_minimum_required(VERSION 3.8)
project(robot_firmware)

find_package(ament_cmake REQUIRED)

# The firmware is flashed to the Arduino, not built by colcon. We only stage
# the sketch and flash helper into the package share/lib so they ship with an
# installed workspace and are discoverable via `ros2 pkg prefix robot_firmware`.
install(
  DIRECTORY arduino
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS flash_firmware.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
