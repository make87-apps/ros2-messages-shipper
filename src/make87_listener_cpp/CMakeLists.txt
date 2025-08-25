cmake_minimum_required(VERSION 3.16)
project(make87_listener_cpp)

# Set policy for FetchContent timestamps
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

# Fetch dependencies
include(FetchContent)

# Fetch nlohmann/json
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)

# Fetch Rerun C++ SDK
FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)

FetchContent_MakeAvailable(nlohmann_json rerun_sdk)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs)
target_link_libraries(listener nlohmann_json::nlohmann_json rerun_sdk OpenSSL::SSL OpenSSL::Crypto)

install(TARGETS listener
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
