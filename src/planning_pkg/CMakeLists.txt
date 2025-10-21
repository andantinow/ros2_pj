cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# --- [ROS2 통신 의존성 추가] ---
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) # /odom 메시지 사용
find_package(nav_msgs REQUIRED)      # /path 메시지 사용
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# -------------------------------

# 헤더 파일 경로 설정 (C++ 코드가 include 디렉토리를 찾도록 함)
include_directories(include)

# 실행 파일 정의 및 의존성 연결
add_executable(path_planner src/simple_path_planner.cpp)
ament_target_dependencies(path_planner rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

# 컴파일된 실행 파일 설치
install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
