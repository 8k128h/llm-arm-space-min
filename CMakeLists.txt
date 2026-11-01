cmake_minimum_required(VERSION 3.8)
project(llm_arm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_place_server src/pick_place_server.cpp)
ament_target_dependencies(pick_place_server
  rclcpp moveit_ros_planning_interface geometry_msgs std_msgs tf2_geometry_msgs)

install(TARGETS pick_place_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/swept_volume_recorder.py scripts/planner.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
