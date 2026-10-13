cmake_minimum_required(VERSION 3.16)
project(dyx_dds_actuator_monitor LANGUAGES CXX)

# This is a *monitor-only* ROS 2 node, NOT an actuator driver.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(dyx_dds_actuator_monitor_node src/dds_actuator_monitor_node.cpp)
target_compile_features(dyx_dds_actuator_monitor_node PRIVATE cxx_std_17)
target_include_directories(dyx_dds_actuator_monitor_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_rpp_cpp17_core/include)
ament_target_dependencies(dyx_dds_actuator_monitor_node rclcpp px4_msgs)

install(TARGETS dyx_dds_actuator_monitor_node DESTINATION lib/${PROJECT_NAME})
ament_package()
