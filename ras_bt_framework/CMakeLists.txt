cmake_minimum_required(VERSION 3.8)
project(ras_bt_framework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE RelWithDebInfo)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(behaviortree_cpp REQUIRED)
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
  config src templates
  DESTINATION
  share/${PROJECT_NAME}
)

include_directories(include)

add_executable(examples  src/examples.cpp )
ament_target_dependencies(examples behaviortree_cpp)

add_executable(executor  src/executor.cpp src/BTExecutor.cpp )
ament_target_dependencies(executor rclcpp behaviortree_cpp ras_interfaces geometry_msgs rclcpp_action std_srvs)


install(TARGETS
  examples
  executor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scripts/
 DESTINATION lib/${PROJECT_NAME}/)

ament_python_install_package(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
