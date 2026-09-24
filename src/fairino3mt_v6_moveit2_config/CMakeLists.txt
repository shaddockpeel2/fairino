cmake_minimum_required(VERSION 3.22)
project(fairino3mt_v6_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(collision_distance_diagnostic
  src/collision_distance_diagnostic.cpp)
ament_target_dependencies(collision_distance_diagnostic
  moveit_ros_planning
  rclcpp)

add_executable(servo_path_guard src/servo_path_guard.cpp)
target_compile_features(servo_path_guard PRIVATE cxx_std_17)
ament_target_dependencies(servo_path_guard
  moveit_ros_planning rclcpp std_msgs std_srvs sensor_msgs moveit_msgs controller_manager_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_servo_path_check test/test_servo_path_check.cpp)
  target_include_directories(test_servo_path_check PRIVATE src)
  ament_target_dependencies(test_servo_path_check moveit_ros_planning rclcpp)
endif()

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/trajectory_control_bridge.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS collision_distance_diagnostic DESTINATION lib/${PROJECT_NAME})
install(TARGETS servo_path_guard DESTINATION lib/${PROJECT_NAME})
