cmake_minimum_required(VERSION 3.8)
project(control_pkg)

# ... (C/C++ standard settings omitted for brevity, but should be kept in the file) ...

find_package(ament_cmake REQUIRED)

# --- [ROS2 Dependency Discovery] ---
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# -----------------------------------

# Define Executable and link dependencies
add_executable(controller src/simple_controller.cpp)
ament_target_dependencies(controller rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

# Install the compiled executable
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
