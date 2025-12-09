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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Path utilities library
add_library(path_utils src/path_utils.cpp)
ament_target_dependencies(path_utils rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(path_utils tf2_geometry_msgs::tf2_geometry_msgs)
target_include_directories(path_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(raceline_server src/raceline_server_node.cpp)
ament_target_dependencies(raceline_server rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(raceline_server path_utils tf2_geometry_msgs::tf2_geometry_msgs)
target_include_directories(raceline_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS raceline_server DESTINATION lib/${PROJECT_NAME})

# CLI to generate raceline.csv from centerline.csv
add_executable(generate_raceline src/generate_raceline.cpp)
ament_target_dependencies(generate_raceline rclcpp)
target_link_libraries(generate_raceline path_utils)
target_include_directories(generate_raceline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS generate_raceline DESTINATION lib/${PROJECT_NAME})

# Install resources only if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    PATTERN ".gitkeep" EXCLUDE)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/data")
  install(DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
    PATTERN ".gitkeep" EXCLUDE)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tracks")
  install(DIRECTORY tracks
    DESTINATION share/${PROJECT_NAME}
    PATTERN ".gitkeep" EXCLUDE)
endif()

# Install headers
install(DIRECTORY include/
  DESTINATION include
  PATTERN ".gitkeep" EXCLUDE)

# Install library
install(TARGETS path_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(path_utils)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

ament_package()
