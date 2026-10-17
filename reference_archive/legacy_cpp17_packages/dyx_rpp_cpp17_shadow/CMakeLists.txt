cmake_minimum_required(VERSION 3.8)
project(dyx_rpp_cpp17_shadow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dyx_rpp_cpp17_core REQUIRED)

add_executable(dyx_rpp_cpp17_shadow_node src/shadow_node.cpp)
ament_target_dependencies(dyx_rpp_cpp17_shadow_node
  rclcpp px4_msgs std_msgs dyx_rpp_cpp17_core)
install(TARGETS dyx_rpp_cpp17_shadow_node DESTINATION lib/${PROJECT_NAME})
ament_package()
