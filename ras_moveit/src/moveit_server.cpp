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

#include "../include/moveit_server.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Moveit Server Init");

MoveitServer::MoveitServer(std::shared_ptr<rclcpp::Node> move_group_node)
    : Node("moveit_server")
        {

        this->declare_parameter("move_group_name", "lite6");
        this->declare_parameter("collision_object_frame", "world");
        this->declare_parameter("base_frame_id", "link_base");
        this->declare_parameter("end_effector_frame_id", "link_eef");

        move_group_name = this->get_parameter("move_group_name").as_string();
        collision_object_frame = this->get_parameter("collision_object_frame").as_string();
        base_frame_id = this->get_parameter("base_frame_id").as_string();
        end_effector_frame_id = this->get_parameter("end_effector_frame_id").as_string();

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node,
            move_group_name
        );


        RCLCPP_INFO(LOGGER, "Node Started");

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
        std::bind(&MoveitServer::sync_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        execute_traj_srv = this->create_service<ras_interfaces::srv::ActionTraj>(
            "trajectory_topic",
            std::bind(&MoveitServer::trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        AddScenePlane();
        }

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
  
  void MoveitServer::set_constraints(const geometry_msgs::msg::Pose::_orientation_type& quat)
  {
    RCLCPP_INFO(LOGGER, "Orientation Constrains Set");

   // Goal constraints - position


    moveit_msgs::msg::Constraints goal_constraints;

    // Goal constraints - orientation
    moveit_msgs::msg::OrientationConstraint ori_constraint;
    ori_constraint.header.stamp = this->get_clock()->now(); // Set the current time
    ori_constraint.header.frame_id = base_frame_id;
    ori_constraint.orientation.x = quat.x;
    ori_constraint.orientation.y = quat.y;
    ori_constraint.orientation.z = quat.z;
    ori_constraint.orientation.w = quat.w;
    ori_constraint.link_name = end_effector_frame_id;
    ori_constraint.absolute_x_axis_tolerance = 0.75;
    ori_constraint.absolute_y_axis_tolerance = 0.75;
    ori_constraint.absolute_z_axis_tolerance = 0.75;
    ori_constraint.weight = 1.0;
    ori_constraint.parameterization = 1.0;
    goal_constraints.orientation_constraints.push_back(ori_constraint);

    move_group_arm->setPathConstraints(goal_constraints);
  }


  bool MoveitServer::Execute(sensor_msgs::msg::JointState target_joints)
  {
    RCLCPP_INFO(this->get_logger(),"function call");

    trajectory_msgs::msg::JointTrajectory trajectory_msg;

    // move_group_arm->clearPathConstraints();
    RCLCPP_INFO(this->get_logger(),"clear constraints");

    move_group_arm->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    
    move_group_arm->setPlannerId("RRTConnectkConfigDefault");

    move_group_arm->setNumPlanningAttempts(5);
    move_group_arm->setPlanningTime(1);
    move_group_arm->setGoalTolerance(0.005);
    move_group_arm->setGoalOrientationTolerance(0.005);
    move_group_arm->setMaxVelocityScalingFactor(0.2);
    move_group_arm->setMaxAccelerationScalingFactor(0.4);
    RCLCPP_INFO(this->get_logger(),"beforeconstraints");


    RCLCPP_INFO(this->get_logger(),"after constraints");
    move_group_arm->setJointValueTarget(target_joints);

    RCLCPP_INFO(this->get_logger(),"after target pose");

    int count = 15;
    for (int i = 0; i < count; i++)
    {
        if (i < count-2)
        {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(),"after target plan");
        trajectory_msg = my_plan.trajectory_.joint_trajectory;
        if (success)
        {
        move_group_arm->execute(my_plan);
        trajectory_pub->publish(trajectory_msg);
        return 1;
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
        return 1;
        }
        else
        {
          return 0;
        }
        }   
    }
  }

  bool MoveitServer::Execute(geometry_msgs::msg::Pose target_pose) {
    RCLCPP_INFO(this->get_logger(),"function call");

    trajectory_msgs::msg::JointTrajectory trajectory_msg;

    // move_group_arm->clearPathConstraints();
    RCLCPP_INFO(this->get_logger(),"clear constraints");

    move_group_arm->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    
    move_group_arm->setPlannerId("RRTConnectkConfigDefault");

    move_group_arm->setNumPlanningAttempts(5);
    move_group_arm->setPlanningTime(1);
    move_group_arm->setGoalTolerance(0.005);
    move_group_arm->setGoalOrientationTolerance(0.005);
    move_group_arm->setMaxVelocityScalingFactor(0.2);
    move_group_arm->setMaxAccelerationScalingFactor(0.4);
    RCLCPP_INFO(this->get_logger(),"beforeconstraints");

    set_constraints(target_pose.orientation);

    RCLCPP_INFO(this->get_logger(),"after constraints");
    move_group_arm->setPoseTarget(target_pose);

    RCLCPP_INFO(this->get_logger(),"after target pose");

    int count = 15;
    for (int i = 0; i < count; i++)
    {
        if (i < count-2)
        {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(),"after target plan");
        trajectory_msg = my_plan.trajectory_.joint_trajectory;
        if (success)
        {
        move_group_arm->execute(my_plan);
        trajectory_pub->publish(trajectory_msg);
        return 1;
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
        return 1;
        }
        else
        {
          return 0;
        }
        }   
    }
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
        config_pose.orientation.x, config_pose.orientation.y, config_pose.orientation.z, config_pose.orientation.w
    );

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
    for (size_t i = 0; i < config_joints.position.size(); ++i) {
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
    RCLCPP_INFO(LOGGER, "Received RotateEffector request to rotate end effector by angle: %f", request->rotation_angle);


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
      RCLCPP_ERROR(LOGGER, "No joint states available. Aborting rotation.");
      response->success = false;
      return;
    }

    int count = 5;

    // set_constraints();
    // Convert joint_angle (std::vector<float>) to std::vector<double>
    std::vector<double> target_joint_values = {joint_angle[2], joint_angle[0], joint_angle[1], joint_angle[3], joint_angle[4], joint_angle[5]+(request->rotation_angle)};

    // Update the last joint (assumed to be joint6) by adding the rotation
    
    // target_joint_values[5] += request->rotation_angle; // Add rotation in radians to joint6
    RCLCPP_INFO(LOGGER, "Updated joint6 angle: %f", target_joint_values[5]);
    

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
        RCLCPP_INFO(LOGGER, "End effector rotation executed successfully.");
        break;
        }   
      }
  }

  void MoveitServer::sync_callback(const std::shared_ptr<ras_interfaces::srv::JointSat::Request> request,
      std::shared_ptr<ras_interfaces::srv::JointSat::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "MoveitServer::sync_callback called");
    std::vector<double> joint_values;
    for (const auto& x : request->joint_state.position)
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

int main(int argc, char **argv) {
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
