cmake_minimum_required(VERSION 3.8)
project(control_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_controller_node
  src/controller_main.cpp
  src/simple_controller.cpp
)

# (에러 원인이었던 target_include_directories 삭제함)
# 같은 폴더(src)에 있는 헤더는 자동으로 찾습니다.

ament_target_dependencies(simple_controller_node
  rclcpp
  nav_msgs
  ackermann_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

# (중요) 링크 에러 방지를 위해 명시적으로 라이브러리 연결
target_link_libraries(simple_controller_node
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS simple_controller_node
  DESTINATION lib/\${PROJECT_NAME}
)

ament_package()
