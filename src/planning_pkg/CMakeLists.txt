cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

# Use C++17 for ROS 2 Humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Existing example planner (if present)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/simple_path_planner.cpp")
  add_executable(simple_path_planner src/simple_path_planner.cpp)
  ament_target_dependencies(simple_path_planner rclcpp nav_msgs geometry_msgs std_msgs)
  install(TARGETS simple_path_planner DESTINATION lib/${PROJECT_NAME})
endif()

# New: raceline server (C++) - publishes Path (+v_ref) once with latched QoS
add_executable(raceline_server src/raceline_server_node.cpp)
ament_target_dependencies(raceline_server rclcpp nav_msgs geometry_msgs std_msgs)
install(TARGETS raceline_server DESTINATION lib/${PROJECT_NAME})

# New: C++ CLI to generate raceline.csv from centerline.csv
add_executable(generate_raceline src/generate_raceline.cpp)
ament_target_dependencies(generate_raceline rclcpp)
install(TARGETS generate_raceline DESTINATION lib/${PROJECT_NAME})

# Install config/data/tracks resources
install(DIRECTORY config data tracks
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE)

ament_package()
