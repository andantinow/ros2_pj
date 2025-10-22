cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# --- [ROS2 Dependency Discovery] ---
# Essential ROS2 client library
find_package(rclcpp REQUIRED)
# Odometry message type
find_package(nav_msgs REQUIRED)
# Pose, Quaternion types
find_package(geometry_msgs REQUIRED)
# TF2 libraries for coordinate transforms
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# -----------------------------------

# Define Executable and link dependencies
# The executable name 'pose_publisher' must match the name used in the launch file.
add_executable(pose_publisher src/simple_position_publisher.cpp)
ament_target_dependencies(pose_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

# Install the compiled executable
install(TARGETS
  pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
