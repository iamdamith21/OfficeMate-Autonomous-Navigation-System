cmake_minimum_required(VERSION 3.8)
project(ltme_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ROS-agnostic LDCP protocol SDK (bundled Asio + RapidJSON). Builds as a
# static library and exports its own include directory (PUBLIC).
set(LDCP_SDK_DIR "lib/ldcp_sdk")
add_subdirectory(${LDCP_SDK_DIR})

add_executable(ltme_node src/ltme_node.cpp)
target_include_directories(ltme_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
ament_target_dependencies(ltme_node rclcpp sensor_msgs std_srvs)
target_link_libraries(ltme_node ldcp_sdk)

install(TARGETS ltme_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
