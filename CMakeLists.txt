cmake_minimum_required(VERSION 3.5)
project(latched_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(latched_pub_node
  src/latched_pub.cpp
  src/latched_pub_main.cpp
)
ament_target_dependencies(latched_pub_node
  rclcpp
  std_msgs
)
install(TARGETS
  latched_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(latched_sub_node
  src/latched_sub.cpp
  src/latched_sub_main.cpp
)
ament_target_dependencies(latched_sub_node
  rclcpp
  std_msgs
)
install(TARGETS
  latched_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
