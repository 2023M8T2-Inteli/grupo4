cmake_minimum_required(VERSION 3.8)
project(ros_turtles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle src/turtles.cpp)
ament_target_dependencies(turtle rclcpp turtlesim geometry_msgs)


install(TARGETS
turtle
DESTINATION lib/${PROJECT_NAME})


ament_package()
