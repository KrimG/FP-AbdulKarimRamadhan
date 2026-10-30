cmake_minimum_required(VERSION 3.8)
project(master)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(master_node src/master_node.cpp)
ament_target_dependencies(master_node rclcpp std_msgs)

install(TARGETS master_node DESTINATION lib/${PROJECT_NAME})
ament_package()