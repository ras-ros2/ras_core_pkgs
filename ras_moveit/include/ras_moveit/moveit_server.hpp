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

#ifndef MOVEIT_SERVER_HPP
#define MOVEIT_SERVER_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ras_interfaces/srv/pose_req.hpp>
#include <ras_interfaces/srv/joint_sat.hpp>
#include <ras_interfaces/srv/load_exp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/point.h>
#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/position_constraint.h>
#include <moveit_msgs/msg/robot_state.h>
#include <moveit_msgs/msg/workspace_parameters.h>
#include <string>
#include <vector>
#include <ras_interfaces/srv/rotate_effector.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <ras_interfaces/srv/joint_req.hpp>
#include <ras_interfaces/srv/action_traj.hpp>

using moveit::planning_interface::MoveGroupInterface;


struct OrientationTolerance
{
  double x;
  double y;
  double z;
};

struct PlanningParameters
{
  OrientationTolerance orientation_tolerance;
  int planning_attempts;
  double planning_time;
  double goal_tolerance;
  double goal_orientation_tolerance;
  double velocity_scaling_factor;
  double acceleration_scaling_factor;
};

class MoveitServer : public rclcpp::Node, public std::enable_shared_from_this<MoveitServer>
{
public:
  MoveitServer(std::shared_ptr<rclcpp::Node> move_group_node);
  ~MoveitServer();
  void move_to_pose_callback(const std::shared_ptr<ras_interfaces::srv::PoseReq::Request> request, std::shared_ptr<ras_interfaces::srv::PoseReq::Response> response);
  void move_to_joint_callback(const std::shared_ptr<ras_interfaces::srv::JointReq::Request> request, std::shared_ptr<ras_interfaces::srv::JointReq::Response> response);
  void rotate_effector_callback(const std::shared_ptr<ras_interfaces::srv::RotateEffector::Request> request, std::shared_ptr<ras_interfaces::srv::RotateEffector::Response> response);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void sync_callback(const std::shared_ptr<ras_interfaces::srv::JointSat::Request> request, std::shared_ptr<ras_interfaces::srv::JointSat::Response> response);
  void set_constraints(const geometry_msgs::msg::Pose::_orientation_type &quat);
  bool Execute(geometry_msgs::msg::Pose target_pose);
  bool Execute(sensor_msgs::msg::JointState target_joints);
  void trajectory_callback(const std::shared_ptr<ras_interfaces::srv::ActionTraj::Request> request,
                           std::shared_ptr<ras_interfaces::srv::ActionTraj::Response> response);
  // void trajectory_callback(const std::shared_ptr<ras_interfaces::srv::ActionTraj::Request> request,
  //   std::shared_ptr<ras_interfaces::srv::ActionTraj::Response> response);
  void AddScenePlane();

private:
  void configure_move_group();
  bool plan_and_execute_with_retries();

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // moveit::core::RobotStatePtr current_state_arm;
  // const moveit::core::JointModelGroup *joint_model_group_arm;
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  rclcpp::Service<ras_interfaces::srv::PoseReq>::SharedPtr move_to_pose_srv_;
  rclcpp::Service<ras_interfaces::srv::JointReq>::SharedPtr move_to_joint_srv_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;
  rclcpp::Service<ras_interfaces::srv::RotateEffector>::SharedPtr rotate_effector_srv_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<ras_interfaces::srv::JointSat>::SharedPtr sync_srv;
  rclcpp::Service<ras_interfaces::srv::ActionTraj>::SharedPtr execute_traj_srv;

  std::vector<float> joint_angle;
  std::string collision_object_frame;
  std::string base_frame_id;
  std::string end_effector_frame_id;
  std::string move_group_name;

  PlanningParameters planning_parameters_;
};

#endif