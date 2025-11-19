cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_model_msgs REQUIRED)

# Optional: detect acados generated solver sources
set(ACADOS_SOLVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/acados_solver")
if(EXISTS "${ACADOS_SOLVER_DIR}")
  message(STATUS "acados solver detected at: ${ACADOS_SOLVER_DIR}")
  add_definitions(-DHAVE_ACADOS)
  file(GLOB ACADOS_SRCS "${ACADOS_SOLVER_DIR}/*.c")
  include_directories(${ACADOS_SOLVER_DIR})
else()
  message(WARNING "acados solver not found; NMPC will use safe fallback controller.")
endif()

# Build simple_controller only if nlohmann/json.hpp header exists
include(CheckIncludeFileCXX)
check_include_file_cxx("nlohmann/json.hpp" HAVE_NLOHMANN_JSON)
if(HAVE_NLOHMANN_JSON)
  add_executable(simple_controller src/simple_controller.cpp)
  ament_target_dependencies(simple_controller rclcpp nav_msgs geometry_msgs ackermann_msgs)
  install(TARGETS simple_controller DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "nlohmann/json.hpp not found; simple_controller will not be built.")
endif()

add_executable(nmpc_engine_node src/nmpc_engine_node.cpp)
target_include_directories(nmpc_engine_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(ACADOS_SRCS)
  target_sources(nmpc_engine_node PRIVATE ${ACADOS_SRCS})
endif()
ament_target_dependencies(nmpc_engine_node rclcpp nav_msgs geometry_msgs ackermann_msgs std_msgs vehicle_model_msgs)
install(TARGETS nmpc_engine_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
