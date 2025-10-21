cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# --- [ROS2 의존성 찾기] ---
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED) # <-- nav_msgs 추가
# -----------------------------

# 실행 파일 정의 및 의존성 연결
add_executable(controller src/simple_controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs nav_msgs) # <-- nav_msgs 추가

# 헤더 파일 경로 설정
include_directories(include)

# 컴파일된 실행 파일 설치
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
