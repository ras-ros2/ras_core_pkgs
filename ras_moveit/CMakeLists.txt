cmake_minimum_required(VERSION 3.8)
project(ras_moveit)

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
find_package(ras_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

install(
  DIRECTORY
  config
  include
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

include_directories(include)

## build executable
add_executable(moveit_server
  src/moveit_server.cpp
  src/moveit_services.cpp
  )

ament_target_dependencies(moveit_server
  rclcpp
  moveit_ros_planning_interface
  trajectory_msgs
  ras_interfaces
  std_srvs
  control_msgs)

install(TARGETS
  moveit_server
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
