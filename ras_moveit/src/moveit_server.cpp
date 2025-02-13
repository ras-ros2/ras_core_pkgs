/*
 *
 * Copyright (C) 2024 Harsh Davda
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * For inquiries or further information, you may contact:
 * Harsh Davda
 * Email: info@opensciencestack.org
 */

#include "ras_moveit/moveit_server.hpp"
// static const rclcpp::this->get_logger() this->get_logger() = rclcpp::get_logger("Moveit Server Init");
MoveitServer::MoveitServer(std::shared_ptr<rclcpp::Node> move_group_node)
    : Node("moveit_server")
{


  this->declare_parameter("move_group_name", "lite6");
  this->declare_parameter("collision_object_frame", "world");
  this->declare_parameter("base_frame_id", "link_base");
  this->declare_parameter("end_effector_frame_id", "link_eef");
  this->declare_parameter<double>("orientation_tolerance.x", 0.75);
  this->declare_parameter<double>("orientation_tolerance.y", 0.75);
  this->declare_parameter<double>("orientation_tolerance.z", 0.75);
  this->declare_parameter<int>("planning_attempts", 5);
  this->declare_parameter<double>("planning_time", 2.0);
  this->declare_parameter<double>("goal_tolerance", 0.005);
  this->declare_parameter<double>("goal_orientation_tolerance", 0.005);
  this->declare_parameter<double>("velocity_scaling_factor", 0.2);
  this->declare_parameter<double>("acceleration_scaling_factor", 0.4);

  planning_parameters_.orientation_tolerance.x = this->get_parameter("orientation_tolerance.x").as_double();
  planning_parameters_.orientation_tolerance.y = this->get_parameter("orientation_tolerance.y").as_double();
  planning_parameters_.orientation_tolerance.z = this->get_parameter("orientation_tolerance.z").as_double();
  planning_parameters_.planning_attempts = this->get_parameter("planning_attempts").as_int();
  planning_parameters_.planning_time = this->get_parameter("planning_time").as_double();
  planning_parameters_.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
  planning_parameters_.goal_orientation_tolerance = this->get_parameter("goal_orientation_tolerance").as_double();
  planning_parameters_.velocity_scaling_factor = this->get_parameter("velocity_scaling_factor").as_double();
  planning_parameters_.acceleration_scaling_factor = this->get_parameter("acceleration_scaling_factor").as_double();

  
  move_group_name = this->get_parameter("move_group_name").as_string();
  RCLCPP_INFO(this->get_logger(), "Movegrup name is %s Acceleration scaling factor: %f", move_group_name.c_str(), planning_parameters_.acceleration_scaling_factor);
  collision_object_frame = this->get_parameter("collision_object_frame").as_string();
  base_frame_id = this->get_parameter("base_frame_id").as_string();
  end_effector_frame_id = this->get_parameter("end_effector_frame_id").as_string();

  move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      move_group_node,
      move_group_name);

  RCLCPP_INFO(this->get_logger(), "Node Started");

  move_to_pose_srv_ = this->create_service<ras_interfaces::srv::PoseReq>(
      "/create_traj",
      std::bind(&MoveitServer::move_to_pose_callback, this, std::placeholders::_1, std::placeholders::_2));

  move_to_joint_srv_ = this->create_service<ras_interfaces::srv::JointReq>(
      "/move_to_joint_states",
      std::bind(&MoveitServer::move_to_joint_callback, this, std::placeholders::_1, std::placeholders::_2));

  rotate_effector_srv_ = this->create_service<ras_interfaces::srv::RotateEffector>(
      "/rotate_effector",
      std::bind(&MoveitServer::rotate_effector_callback, this, std::placeholders::_1, std::placeholders::_2));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&MoveitServer::joint_state_callback, this, std::placeholders::_1));

  trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("trajectory_topic", 10);

  sync_srv = this->create_service<ras_interfaces::srv::JointSat>(
      "/sync_arm",
      std::bind(&MoveitServer::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

  execute_traj_srv = this->create_service<ras_interfaces::srv::ActionTraj>(
      "trajectory_topic",
      std::bind(&MoveitServer::trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));

  AddScenePlane();
}

MoveitServer::~MoveitServer()
{
}

void MoveitServer::AddScenePlane()
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = collision_object_frame;
  collision_object.id = "ground_plane";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 10;
  primitive.dimensions[primitive.BOX_Y] = 10;
  primitive.dimensions[primitive.BOX_Z] = 0.01;
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.05;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObject(collision_object);
  RCLCPP_INFO(this->get_logger(), "Added plane to planning scene");
}

void MoveitServer::set_constraints(const geometry_msgs::msg::Pose::_orientation_type &quat)
{
  RCLCPP_INFO(this->get_logger(), "Orientation Constrains Set");

  // Goal constraints - position
  moveit_msgs::msg::Constraints goal_constraints;

  // Goal constraints - orientation
  moveit_msgs::msg::OrientationConstraint ori_constraint;
  ori_constraint.header.stamp = this->get_clock()->now();
  ori_constraint.header.frame_id = base_frame_id;
  ori_constraint.orientation.x = quat.x;
  ori_constraint.orientation.y = quat.y;
  ori_constraint.orientation.z = quat.z;
  ori_constraint.orientation.w = quat.w;
  ori_constraint.link_name = end_effector_frame_id;
  ori_constraint.absolute_x_axis_tolerance = planning_parameters_.orientation_tolerance.x;
  ori_constraint.absolute_y_axis_tolerance = planning_parameters_.orientation_tolerance.y;
  ori_constraint.absolute_z_axis_tolerance = planning_parameters_.orientation_tolerance.z;
  ori_constraint.weight = 1.0;
  ori_constraint.parameterization = 1.0;
  goal_constraints.orientation_constraints.push_back(ori_constraint);

  move_group_arm->setPathConstraints(goal_constraints);
}

bool MoveitServer::Execute(sensor_msgs::msg::JointState target_joints)
{
  // move_group_arm->clearPathConstraints();
  // RCLCPP_INFO(this->get_logger(), "clear constraints");
  configure_move_group();
  move_group_arm->setJointValueTarget(target_joints);
  return plan_and_execute_with_retries();
}

bool MoveitServer::Execute(geometry_msgs::msg::Pose target_pose)
{
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  // move_group_arm->clearPathConstraints();
  RCLCPP_INFO(this->get_logger(), "clear constraints");
  
  configure_move_group();
  set_constraints(target_pose.orientation);
  move_group_arm->setPoseTarget(target_pose);
  return plan_and_execute_with_retries();
}

void MoveitServer::configure_move_group()
{
  move_group_arm->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
  move_group_arm->setPlannerId("RRTConnectkConfigDefault");
  move_group_arm->setNumPlanningAttempts(planning_parameters_.planning_attempts);
  move_group_arm->setPlanningTime(planning_parameters_.planning_time);
  move_group_arm->setGoalTolerance(planning_parameters_.goal_tolerance);
  move_group_arm->setGoalOrientationTolerance(planning_parameters_.goal_orientation_tolerance);
  move_group_arm->setMaxVelocityScalingFactor(planning_parameters_.velocity_scaling_factor);
  move_group_arm->setMaxAccelerationScalingFactor(planning_parameters_.acceleration_scaling_factor);
}

bool MoveitServer::plan_and_execute_with_retries()
{
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  int count = 15;
  for (int i = 0; i < count; i++)
  {
    if (i < count - 2)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan.trajectory_.joint_trajectory;
      if (success)
      {
        move_group_arm->execute(my_plan);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
    }
    else
    {
      move_group_arm->clearPathConstraints();
      RCLCPP_INFO(this->get_logger(), "Clearning Constraints");
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      bool success2 = (move_group_arm->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      trajectory_msg = my_plan2.trajectory_.joint_trajectory;
      if (success2)
      {
        move_group_arm->execute(my_plan2);
        trajectory_pub->publish(trajectory_msg);
        break;
      }
      else
      {
        return 0;
      }
    }
  }
  // TODO(Sachin): check if the goal executed successfully
  return 1;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto move_group_node =
      rclcpp::Node::make_shared("moveit_server");
  rclcpp::executors::SingleThreadedExecutor planner_executor;
  auto planner_node = std::make_shared<MoveitServer>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}
