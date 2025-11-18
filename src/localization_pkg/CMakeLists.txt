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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vehicle_model_msgs REQUIRED)

add_executable(estimator_node src/estimator_node.cpp)
ament_target_dependencies(estimator_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs vehicle_model_msgs
)

install(TARGETS estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
