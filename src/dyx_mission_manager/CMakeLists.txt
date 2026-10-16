cmake_minimum_required(VERSION 3.16)
project(dyx_mission_manager LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dyx_rpp_cpp17_core REQUIRED)
add_executable(dyx_mission_manager_node src/dyx_mission_manager_node.cpp)
target_include_directories(dyx_mission_manager_node PRIVATE include)
ament_target_dependencies(dyx_mission_manager_node rclcpp std_msgs geometry_msgs dyx_rpp_cpp17_core)
install(TARGETS dyx_mission_manager_node DESTINATION lib/${PROJECT_NAME})

add_executable(
  dyx_mission_lifecycle_test_node
  src/dyx_mission_lifecycle_test_node.cpp
)

ament_target_dependencies(
  dyx_mission_lifecycle_test_node
  rclcpp
  std_msgs
  dyx_rpp_cpp17_core
)

install(
  TARGETS dyx_mission_lifecycle_test_node
  DESTINATION lib/${PROJECT_NAME}
)


find_package(px4_msgs REQUIRED)

add_executable(
  dyx_mission_dds_runtime_node
  src/dyx_mission_dds_runtime_node.cpp
)

ament_target_dependencies(
  dyx_mission_dds_runtime_node
  rclcpp
  px4_msgs
  std_msgs
  dyx_rpp_cpp17_core
)

install(
  TARGETS dyx_mission_dds_runtime_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
