cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp nav_msgs geometry_msgs ackermann_msgs)

install(TARGETS simple_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
