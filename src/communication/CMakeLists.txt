cmake_minimum_required(VERSION 3.8)
project(communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(sender src/sender.cpp)
ament_target_dependencies(sender rclcpp std_msgs)

add_executable(receiver src/receiver.cpp)
ament_target_dependencies(receiver rclcpp std_msgs)

add_executable(keyboard_teleop src/keyboard_teleop_node.cpp)
ament_target_dependencies(keyboard_teleop rclcpp std_msgs)

install(TARGETS sender receiver keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
