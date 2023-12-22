cmake_minimum_required(VERSION 3.8)
project(sample_sensing_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensing_msgs REQUIRED)

set(dependencies
  std_msgs
  rclcpp
  sensing_msgs
)

add_executable(sample_publisher src/sample_publisher.cpp)
add_executable(zx120_end_effector_publisher src/zx120_end_effector_publisher.cpp)

ament_target_dependencies(sample_publisher ${dependencies})
ament_target_dependencies(zx120_end_effector_publisher ${dependencies})

install(TARGETS
  sample_publisher
  zx120_end_effector_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
