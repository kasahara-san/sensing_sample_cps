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

add_executable(zx200_collision_objects_ic120 src/zx200_collision_objects_ic120.cpp)
add_executable(zx200_end_effector_publisher src/zx200_end_effector_publisher.cpp)
add_executable(flgs_publisher src/flgs_publisher.cpp)

ament_target_dependencies(zx200_collision_objects_ic120 ${dependencies})
ament_target_dependencies(zx200_end_effector_publisher ${dependencies})
ament_target_dependencies(flgs_publisher ${dependencies})

install(TARGETS
  zx200_collision_objects_ic120
  zx200_end_effector_publisher
  flgs_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
