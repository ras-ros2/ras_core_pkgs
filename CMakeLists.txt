cmake_minimum_required(VERSION 3.8)
project(oss_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(control_msgs REQUIRED)
find_package(oss_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

install(
  DIRECTORY
  include
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

include_directories(include)

add_executable(moveit_server src/moveit_server.cpp)

ament_target_dependencies(moveit_server
  rclcpp
  moveit_ros_planning_interface
  trajectory_msgs
  oss_interfaces
  std_srvs
  control_msgs)

add_executable(moveit_real_server src/moveit_real_server.cpp)

ament_target_dependencies(moveit_real_server
  rclcpp
  moveit_ros_planning_interface
  trajectory_msgs
  oss_interfaces
  std_srvs
  control_msgs)
  

install(TARGETS
  moveit_server
  moveit_real_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
