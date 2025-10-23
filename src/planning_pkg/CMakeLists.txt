cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# --- [ROS2 Dependency Discovery] ---
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# -----------------------------

# Define Executable and link dependencies
# Corrected: Source file name matches the actual file (simple_path_planner.cpp)
add_executable(path_planner src/simple_path_planner.cpp)
ament_target_dependencies(path_planner rclcpp geometry_msgs nav_msgs)

# Install the compiled executable
install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
