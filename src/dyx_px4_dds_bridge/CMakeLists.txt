cmake_minimum_required(VERSION 3.8)
project(dyx_px4_dds_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rpp_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dyx_px4_dds_bridge_node src/dds_bridge_node.cpp)
target_include_directories(dyx_px4_dds_bridge_node PRIVATE include)
ament_target_dependencies(dyx_px4_dds_bridge_node
  rclcpp px4_msgs rpp_interfaces std_msgs
)

install(TARGETS dyx_px4_dds_bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()
