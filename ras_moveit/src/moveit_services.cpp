/*
 *
 * Copyright (C) 2025 Sachin Kumar
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
 * Sachin Kumar
 * Email: info@opensciencestack.org
 */

#include "ras_moveit/moveit_server.hpp"

void MoveitServer::trajectory_callback(const std::shared_ptr<ras_interfaces::srv::ActionTraj::Request> request,
                                       std::shared_ptr<ras_interfaces::srv::ActionTraj::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received trajectory message");

    moveit_msgs::msg::RobotTrajectory robot_trajectory;

    // Convert JointTrajectory to RobotTrajectory
    robot_trajectory.joint_trajectory = request->traj;

    bool success = (move_group_arm->execute(robot_trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    response->success = 1;
}

void MoveitServer::move_to_pose_callback(
    const std::shared_ptr<ras_interfaces::srv::PoseReq::Request> request,
    std::shared_ptr<ras_interfaces::srv::PoseReq::Response> response)
{
    RCLCPP_WARN(this->get_logger(), "Received Pose request...");

    geometry_msgs::msg::Pose config_pose;
    config_pose = request->object_pose;

    RCLCPP_INFO(this->get_logger(),
                "Received pose:\n"
                "Position: x=%f, y=%f, z=%f\n"
                "Orientation: x=%f, y=%f, z=%f, w=%f",
                config_pose.position.x, config_pose.position.y, config_pose.position.z,
                config_pose.orientation.x, config_pose.orientation.y, config_pose.orientation.z, config_pose.orientation.w);

    bool status = Execute(config_pose);

    response->success = status;
}

void MoveitServer::move_to_joint_callback(
    const std::shared_ptr<ras_interfaces::srv::JointReq::Request> request,
    std::shared_ptr<ras_interfaces::srv::JointReq::Response> response)
{
    RCLCPP_WARN(this->get_logger(), "Received Joint request...");

    sensor_msgs::msg::JointState config_joints;
    config_joints = request->joints;

    std::ostringstream joint_info;
    joint_info << "Received joints:\n";
    for (size_t i = 0; i < config_joints.position.size(); ++i)
    {
        joint_info << "Joint" << (i + 1) << ": " << config_joints.position[i] << "\n";
    }
    RCLCPP_INFO(this->get_logger(), joint_info.str().c_str());

    bool status = Execute(config_joints);

    response->success = status;
}

void MoveitServer::rotate_effector_callback(
    const std::shared_ptr<ras_interfaces::srv::RotateEffector::Request> request,
    std::shared_ptr<ras_interfaces::srv::RotateEffector::Response> response)
{
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    RCLCPP_INFO(this->get_logger(), "Received RotateEffector request to rotate end effector by angle: %f", request->rotation_angle);

    move_group_arm->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

    move_group_arm->setPlannerId("RRTConnectkConfigDefault");

    move_group_arm->setNumPlanningAttempts(5);
    move_group_arm->setPlanningTime(2);
    move_group_arm->setGoalTolerance(0.005);
    move_group_arm->setGoalOrientationTolerance(0.005);
    move_group_arm->setMaxVelocityScalingFactor(0.4);
    move_group_arm->setMaxAccelerationScalingFactor(0.4);
    move_group_arm->clearPathConstraints();

    // Ensure joint states are available
    if (joint_angle.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No joint states available. Aborting rotation.");
        response->success = false;
        return;
    }

    int count = 5;

    // set_constraints();
    // Convert joint_angle (std::vector<float>) to std::vector<double>
    std::vector<double> target_joint_values = {joint_angle[2], joint_angle[0], joint_angle[1], joint_angle[3], joint_angle[4], joint_angle[5] + (request->rotation_angle)};

    // Update the last joint (assumed to be joint6) by adding the rotation

    // target_joint_values[5] += request->rotation_angle; // Add rotation in radians to joint6
    RCLCPP_INFO(this->get_logger(), "Updated joint6 angle: %f", target_joint_values[5]);

    for (int i = 0; i < count; i++)
    {
        move_group_arm->setJointValueTarget(target_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
        bool success = (move_group_arm->plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);
        trajectory_msg = my_plan3.trajectory_.joint_trajectory;

        if (success)
        {
            move_group_arm->execute(my_plan3);
            trajectory_pub->publish(trajectory_msg);
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "End effector rotation executed successfully.");
            break;
        }
    }
}

void MoveitServer::sync_callback(const std::shared_ptr<ras_interfaces::srv::JointSat::Request> request,
                                 std::shared_ptr<ras_interfaces::srv::JointSat::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "MoveitServer::sync_callback called");
    std::vector<double> joint_values;
    for (const auto &x : request->joint_state.position)
    {
        joint_values.push_back(x);
    }

    move_group_arm->setJointValueTarget(joint_values);
    move_group_arm->setMaxVelocityScalingFactor(0.7);
    move_group_arm->setMaxAccelerationScalingFactor(0.7);
    // testing purpose
    move_group_arm->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

    // move_group_arm->setPlannerId("RRTConnectkConfigDefault");

    move_group_arm->setNumPlanningAttempts(5);
    move_group_arm->setPlanningTime(2);
    move_group_arm->setGoalTolerance(0.01);
    move_group_arm->setGoalOrientationTolerance(0.035);
    move_group_arm->clearPathConstraints();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool result = (move_group_arm->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (result)
    {
        result = (move_group_arm->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (result)
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");
            response->successq = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory not executed");
            response->successq = false;
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Planning for trajectory failed");
        response->successq = false;
    }
}

void MoveitServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) // <-- Updated this line
{
    joint_angle.clear();
    for (auto i : msg->position)
    {
        joint_angle.push_back(i);
    }
}