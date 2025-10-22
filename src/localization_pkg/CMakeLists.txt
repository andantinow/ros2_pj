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
# -----------------------------

# 실행 파일 정의 및 의존성 연결
add_executable(odom_subscriber src/simple_position_publisher.cpp)
ament_target_dependencies(odom_subscriber rclcpp nav_msgs geometry_msgs)

# 컴파일된 실행 파일 설치
install(TARGETS
  odom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
