cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ROS2 빌드 시스템
find_package(ament_cmake REQUIRED)

# --- [ROS2 의존성 추가] ---
# ROS C++ 코어
find_package(rclcpp REQUIRED)
# 위치 메시지 (PoseStamped)
find_package(geometry_msgs REQUIRED)
# TF2 쿼터니언 변환 헬퍼 (tf2::toMsg 사용)
find_package(tf2_geometry_msgs REQUIRED) 
# -----------------------------

# 실행 파일 생성
add_executable(pose_publisher src/simple_position_publisher.cpp)

# 실행 파일의 의존성 연결
ament_target_dependencies(pose_publisher rclcpp geometry_msgs tf2_geometry_msgs)

# 헤더 파일 경로 설정 (C++ 코드가 include 디렉토리를 찾도록 함)
include_directories(include)

# 컴파일된 실행 파일 설치
install(TARGETS
  pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
