cmake_minimum_required(VERSION 3.16)
project(ur3_letter_drawer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(draw_letter_n src/draw_letter_n.cpp)
target_compile_features(draw_letter_n PUBLIC cxx_std_17)
ament_target_dependencies(
  draw_letter_n
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  visualization_msgs
)

add_executable(draw_circle src/draw_circle.cpp)
target_compile_features(draw_circle PUBLIC cxx_std_17)
ament_target_dependencies(
  draw_circle
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  visualization_msgs
)

install(TARGETS draw_letter_n draw_circle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
