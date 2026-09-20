cmake_minimum_required(VERSION 3.8)
project(fairino_mtc_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_node
  moveit_task_constructor_core
  rclcpp
)

add_executable(grasp_scene_node src/grasp_scene_node.cpp)
target_compile_features(grasp_scene_node PUBLIC cxx_std_17)
ament_target_dependencies(
  grasp_scene_node
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  shape_msgs
)

add_executable(first_grasp_node src/first_grasp_node.cpp)
target_compile_features(first_grasp_node PUBLIC cxx_std_17)
ament_target_dependencies(
  first_grasp_node
  control_msgs
  fairino_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
  std_srvs
)

add_compile_options(-g)

install(TARGETS mtc_node grasp_scene_node first_grasp_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/p6_preflight
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_p6_checks test/test_p6_checks.py)
  ament_add_pytest_test(test_sort_regions test/test_sort_regions.py)
  ament_add_pytest_test(test_place_zone test/test_place_zone.py)
  ament_add_pytest_test(test_stack_task test/test_stack_task.py)
  ament_add_pytest_test(test_idle_cancel test/test_idle_cancel.py)
  ament_add_pytest_test(test_source_height test/test_source_height.py)
  ament_add_pytest_test(test_release_hold test/test_release_hold.py)
  ament_add_pytest_test(
    test_candidate_duration_selection
    test/test_candidate_duration_selection.py)
endif()

ament_package()
