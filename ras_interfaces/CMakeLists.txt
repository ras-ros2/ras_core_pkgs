cmake_minimum_required(VERSION 3.5)

project(ras_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/ObjectState.msg"
  "msg/Instruction.msg"
  "msg/BTNodeStatus.msg"
  "msg/BTGenMapping.msg"
)
set(srv_files
  "srv/PrimitiveExec.srv"
  "srv/PoseReq.srv"
  "srv/JointReq.srv"
  "srv/LoadExp.srv"
  "srv/AddObject.srv"
  "srv/StatusLog.srv"
  "srv/JointSat.srv"
  "srv/TrajSec.srv"
  "srv/SetPath.srv"
  "srv/TrajSend.srv"
  "srv/PlayPath.srv"
  "srv/ReadBlack.srv"
  "srv/RotateEffector.srv"
  "srv/ActionTraj.srv"
  "srv/GripperLog.srv"
  "srv/TrajLog.srv"
  "srv/ArucoPoses.srv"
  "srv/MoveSim.srv"
  "srv/SpawnSim.srv"
)
set(action_files
  "action/BTInterface.action"
  "action/BTPrimitive.action"
  "action/ExecuteTraj.action"
  "action/ExecuteExp.action"
  "action/BTGenerative.action"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs moveit_msgs trajectory_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()