cmake_minimum_required(VERSION 3.8)
project(control_pkg)

# Default to C++17 for better standard library support
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Check for acados (optional dependency)
# If acados is installed, define ACADOS_AVAILABLE
find_package(acados QUIET)
if(acados_FOUND)
  message(STATUS "acados found - using real acados solver")
  add_definitions(-DACADOS_AVAILABLE)
else()
  message(STATUS "acados not found - using fallback solver")
endif()

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

# ============================================================================
# NMPC Lifecycle Node (acados-ready, LifecycleNode pattern)
# ============================================================================
# This is the refactored NMPC controller that addresses:
# - Issue 1.x: acados integration (when available)
# - Issue 2.1: LifecycleNode pattern for proper initialization
# - Issue 2.2: MultiThreadedExecutor with callback groups
# ============================================================================

# Fallback NMPC Solver library (always built)
add_library(fallback_nmpc_solver
  src/fallback_nmpc_solver.cpp
)

target_include_directories(fallback_nmpc_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(fallback_nmpc_solver
  Eigen3::Eigen
)

# NMPC Lifecycle Node
add_executable(nmpc_lifecycle_node
  src/nmpc_lifecycle_node.cpp
)

target_include_directories(nmpc_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(nmpc_lifecycle_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  nav_msgs
  ackermann_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(nmpc_lifecycle_node
  Eigen3::Eigen
  fallback_nmpc_solver
)

# If acados is available, link it
if(acados_FOUND)
  target_link_libraries(nmpc_lifecycle_node
    acados::acados
  )
endif()

# ============================================================================
# Collision Recovery Node (Stop-Wait-Reverse FSM)
# ============================================================================
# Implements deterministic collision recovery for F1TENTH
# Takes highest priority in the Ackermann MUX during emergency
# ============================================================================

add_executable(collision_recovery_node
  src/collision_recovery_node.cpp
)

target_include_directories(collision_recovery_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(collision_recovery_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  std_msgs
)

# ============================================================================
# Adaptive Cruise Control Node (Time-Headway based ACC)
# ============================================================================
# Implements velocity-synchronized following with relative velocity estimation
# ============================================================================

find_package(std_msgs REQUIRED)

add_executable(adaptive_cruise_control_node
  src/adaptive_cruise_control_node.cpp
)

target_include_directories(adaptive_cruise_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(adaptive_cruise_control_node
  rclcpp
  ackermann_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS 
  simple_controller_node
  nmpc_engine_node
  nmpc_lifecycle_node
  collision_recovery_node
  adaptive_cruise_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install library
install(TARGETS fallback_nmpc_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts for acados code generation
install(PROGRAMS
  scripts/generate_acados_ocp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
