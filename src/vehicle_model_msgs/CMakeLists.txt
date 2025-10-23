cmake_minimum_required(VERSION 3.8)
project(vehicle_model_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate C++ headers and necessary code for the message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdaptiveVehicleModel.msg"
  DEPENDENCIES std_msgs
)

# Install the message files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
