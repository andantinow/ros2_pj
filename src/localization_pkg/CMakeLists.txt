cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# localization_pkg/CMakeLists.txt 파일에 추가

# 1. 의존 패키지 찾기: geometry_msgs 헤더 파일을 사용하기 위해 필요
find_package(geometry_msgs REQUIRED)

# 2. 실행 파일 생성: simple_position_publisher.cpp를 컴파일하여 'pose_publisher'라는 실행 파일을 만듭니다.
add_executable(pose_publisher src/simple_position_publisher.cpp)

# 3. 실행 파일의 의존성 연결: 실행 파일이 rclcpp와 geometry_msgs의 기능을 사용하도록 연결
ament_target_dependencies(pose_publisher rclcpp geometry_msgs)

# 4. 설치 명령: 'pose_publisher' 실행 파일을 빌드 후 시스템에 복사합니다.
install(TARGETS
  pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
