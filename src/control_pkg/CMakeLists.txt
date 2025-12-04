cmake_minimum_required(VERSION 3.8)
project(control_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Simple Controller Node
add_executable(simple_controller_node
  src/controller_main.cpp
  src/simple_controller.cpp
)

target_include_directories(simple_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simple_controller_node
  rclcpp
  nav_msgs
  ackermann_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(simple_controller_node
  tf2_geometry_msgs::tf2_geometry_msgs
)

# NMPC Engine Node
add_executable(nmpc_engine_node
  src/nmpc_engine_node.cpp
)

target_include_directories(nmpc_engine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(nmpc_engine_node
  rclcpp
  nav_msgs
  ackermann_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(nmpc_engine_node
  Eigen3::Eigen
)

install(TARGETS 
  simple_controller_node
  nmpc_engine_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
