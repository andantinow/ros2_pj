cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vehicle_model_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Estimator Node (existing)
add_executable(estimator_node src/estimator_node.cpp)
ament_target_dependencies(estimator_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs vehicle_model_msgs
)

# Scan Processor Node - LiDAR scan processing for localization
add_executable(scan_processor_node src/scan_processor_node.cpp)
ament_target_dependencies(scan_processor_node
  rclcpp sensor_msgs geometry_msgs std_msgs nav_msgs
)

# Dual EKF Node - Two-stage EKF for robust state estimation
add_executable(dual_ekf_node src/dual_ekf_node.cpp)
ament_target_dependencies(dual_ekf_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)
target_include_directories(dual_ekf_node PRIVATE ${EIGEN3_INCLUDE_DIR})

# Install executables
install(TARGETS 
  estimator_node
  scan_processor_node
  dual_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
