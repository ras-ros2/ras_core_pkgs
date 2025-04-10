cmake_minimum_required(VERSION 3.8)
project(ras_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ras_interfaces REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_executable(spawn_model_node src/spawn_model_node.cc)
ament_target_dependencies(spawn_model_node rclcpp geometry_msgs std_msgs ras_interfaces)

target_link_libraries(spawn_model_node 
  ignition-plugin1::ignition-plugin1
  ignition-gazebo6::ignition-gazebo6
  #libstatistics_collector
)

install(TARGETS
        spawn_model_node 
        DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY gazebo_models launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY scripts/
 DESTINATION lib/${PROJECT_NAME}/)
 
ament_python_install_package(${PROJECT_NAME})

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

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
