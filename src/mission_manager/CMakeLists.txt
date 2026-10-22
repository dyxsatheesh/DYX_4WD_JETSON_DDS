cmake_minimum_required(VERSION 3.16)
project(mission_manager LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rpp_interfaces REQUIRED)
find_package(dyx_rpp_cpp17_core REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)
add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(mission_manager_node rclcpp std_msgs rpp_interfaces dyx_rpp_cpp17_core px4_msgs)
target_include_directories(mission_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_mission_manager/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_rpp_cpp17_core/include
)
target_link_libraries(mission_manager_node Boost::system Threads::Threads OpenSSL::Crypto SQLite::SQLite3)
install(TARGETS mission_manager_node DESTINATION lib/${PROJECT_NAME})
ament_package()

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_mission_safety
    test/test_mission_safety.cpp)
  target_include_directories(test_mission_safety PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_mission_manager/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_rpp_cpp17_core/include
  )
  add_test(NAME mission_safety_regression
    COMMAND test_mission_safety)
endif()

if(BUILD_TESTING)
  add_executable(test_mission_dispatch
    test/test_mission_dispatch.cpp)
  target_include_directories(test_mission_dispatch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_mission_manager/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dyx_rpp_cpp17_core/include
  )
  add_test(NAME mission_dispatch_regression
    COMMAND test_mission_dispatch)
endif()
