cmake_minimum_required(VERSION 3.22)
project(fairino3mt_v6_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(collision_distance_diagnostic
  src/collision_distance_diagnostic.cpp)
ament_target_dependencies(collision_distance_diagnostic
  moveit_ros_planning
  rclcpp)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/trajectory_control_bridge.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS collision_distance_diagnostic DESTINATION lib/${PROJECT_NAME})
