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
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # <-- Sensor_msgs 추가
# -----------------------------

# Define Executable and link dependencies
add_executable(odom_subscriber src/simple_position_publisher.cpp)
ament_target_dependencies(odom_subscriber rclcpp nav_msgs geometry_msgs sensor_msgs) # <-- Sensor_msgs 연결

# Install the compiled executable
install(TARGETS
  odom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
