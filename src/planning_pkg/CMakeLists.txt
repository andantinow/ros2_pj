cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

# ... (C/C++ standard settings omitted for brevity, but should be kept in the file) ...

find_package(ament_cmake REQUIRED)

# --- [ROS2 Dependency Discovery] ---
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# -----------------------------------

# Define Executable and link dependencies
add_executable(path_planner src/simple_path_planner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs geometry_msgs)

# Install the compiled executable
install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
