cmake_minimum_required(VERSION 3.8)
project(pick_place_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(pick_place src/pick_place.cpp)
target_compile_features(pick_place PUBLIC cxx_std_17)
ament_target_dependencies(pick_place
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
)

install(TARGETS pick_place DESTINATION lib/${PROJECT_NAME})

ament_package()
