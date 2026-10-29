cmake_minimum_required(VERSION 3.8)
project(px4_ab_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(ab_mission src/ab_mission.cpp)
ament_target_dependencies(ab_mission rclcpp rosgraph_msgs px4_msgs)

install(TARGETS
  ab_mission
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
