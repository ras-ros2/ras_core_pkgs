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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
        
        place_object_srv_ = this->create_service<ras_interfaces::srv::PlaceObject>(
            "/place_object",
            std::bind(&MoveitServer::place_object_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        pick_object_srv_ = this->create_service<ras_interfaces::srv::PickObject>(
            "/pick_object",
            std::bind(&MoveitServer::pick_object_callback, this, std::placeholders::_1, std::placeholders::_2));

        pick_front_srv_ = this->create_service<ras_interfaces::srv::PickFront>(
            "/pick_front",
            std::bind(&MoveitServer::pick_front_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        pick_right_srv_ = this->create_service<ras_interfaces::srv::PickRight>(
            "/pick_right",
            std::bind(&MoveitServer::pick_right_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        pick_left_srv_ = this->create_service<ras_interfaces::srv::PickLeft>(
            "/pick_left",
            std::bind(&MoveitServer::pick_left_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        pick_rear_srv_ = this->create_service<ras_interfaces::srv::PickRear>(
            "/pick_rear",
            std::bind(&MoveitServer::pick_rear_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        AddScenePlane();
        }

  void MoveitServer::trajectory_callback(const std::shared_ptr<ras_interfaces::srv::ActionTraj::Request> request,
    std::shared_ptr<ras_interfaces::srv::ActionTraj::Response> response)
  {
  RCLCPP_INFO(this->get_logger(), "Received trajectory message");

  moveit_msgs::msg::RobotTrajectory robot_trajectory;

  // Convert JointTrajectory to RobotTrajectory
  robot_trajectory.joint_trajectory = request->traj;
    // move_group_arm->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    
    // move_group_arm->setPlannerId("RRTConnectkConfigDefault");

    move_group_arm->setNumPlanningAttempts(5);
    move_group_arm->setPlanningTime(3.5);
    move_group_arm->setGoalTolerance(0.003);
    move_group_arm->setGoalOrientationTolerance(0.003);
    // move_group_arm->setMaxVelocityScalingFactor(0.2);
    // move_group_arm->setMaxAccelerationScalingFactor(0.4);

  bool status = (move_group_arm->execute(robot_trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  response->success = status;
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
    box_pose.position.z = -0.10;
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
    move_group_arm->setPlanningTime(3);
    move_group_arm->setGoalTolerance(0.003);
    move_group_arm->setGoalOrientationTolerance(0.003);
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
        // trajectory_msg = my_plan.trajectory_.joint_trajectory;
        if (success)
        {
        move_group_arm->execute(my_plan);
        // trajectory_pub->publish(trajectory_msg);
        return 1;
        }
        }
        else
        {
        move_group_arm->clearPathConstraints();
        RCLCPP_INFO(this->get_logger(), "Clearning Constraints");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
        bool success2 = (move_group_arm->plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // trajectory_msg = my_plan2.trajectory_.joint_trajectory;
        if (success2)
        {
        move_group_arm->execute(my_plan2);
        // trajectory_pub->publish(trajectory_msg);
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
    int plan_counts = 5;
    float goal_tolerance = 0.005;
    move_group_arm->setNumPlanningAttempts(plan_counts);
    move_group_arm->setPlanningTime(3);
    move_group_arm->setGoalTolerance(goal_tolerance);
    move_group_arm->setGoalOrientationTolerance(0.005);
    move_group_arm->setMaxVelocityScalingFactor(0.2);
    move_group_arm->setMaxAccelerationScalingFactor(0.4);
    RCLCPP_INFO(this->get_logger(),"beforeconstraints");

    set_constraints(target_pose.orientation);

    RCLCPP_INFO(this->get_logger(),"after constraints");
    move_group_arm->setPoseTarget(target_pose);

    RCLCPP_INFO(this->get_logger(),"after target pose");
    int count = plan_counts;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
    double best_cost = std::numeric_limits<double>::max();
    double best_goal_error = std::numeric_limits<double>::max();
    moveit::planning_interface::MoveGroupInterface::Plan best_plan;
    const double cost_difference_threshold = 0.35; 
    int similar_cost_count = 0;
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group_arm->getRobotModel()));

    for (int i = 0; i < count; i++)
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (success)
      {
        plans.push_back(my_plan);
        double cost = 0.0;
        double goal_error = 0.0;

        if (!my_plan.trajectory_.joint_trajectory.points.empty())
        {
          cost = my_plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
            my_plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;
            
            auto last_point = my_plan.trajectory_.joint_trajectory.points.back();
            robot_state->setVariablePositions(last_point.positions);
            robot_state->update();

            const std::string end_effector_link = move_group_arm->getEndEffectorLink();
            const auto& ee_pose = robot_state->getGlobalLinkTransform(end_effector_link);
            const auto& target_position = target_pose.position;
            const auto& ee_position = ee_pose.translation();

            goal_error = std::pow(target_position.x - ee_position.x(), 2) +
              std::pow(target_position.y - ee_position.y(), 2) +
              std::pow(target_position.z - ee_position.z(), 2);
        }
        cost += 0.1 * my_plan.trajectory_.joint_trajectory.points.size();

        RCLCPP_INFO(this->get_logger(), "Plan %d -> Cost: %f, Goal Error: %f", i, cost, goal_error);

        if (cost < best_cost || (cost - best_cost < cost_difference_threshold && goal_error < best_goal_error))
        {
          if (best_cost - cost < cost_difference_threshold)
          {
            similar_cost_count++;
          }
          else
          {
            similar_cost_count = 0; 
          }
          best_cost = cost;
          best_goal_error = goal_error;
          best_plan = my_plan;
        }
        if (similar_cost_count >= 3)
        {
          RCLCPP_INFO(this->get_logger(), "Stopping early: Similar cost plans detected.");
          break;
        }
        if(plans.size() >= 3)
        {
          RCLCPP_INFO(this->get_logger(), "Stopping early: 3 plans found.");
          break;
        }
      } else if ((plans.size() < 3) && (i == count - 3))
      {
      move_group_arm->clearPathConstraints();
      }
    }
    if (!plans.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Best plan selected with cost: %f", best_cost);

        move_group_arm->execute(best_plan);
        trajectory_msg = best_plan.trajectory_.joint_trajectory;
        trajectory_pub->publish(trajectory_msg);
        return 1;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No valid plans found.");
        return 0;
    }


    return 0;
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
    move_group_arm->setGoalTolerance(0.001);
    move_group_arm->setGoalOrientationTolerance(0.001);
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

  void MoveitServer::place_object_callback(
      const std::shared_ptr<ras_interfaces::srv::PlaceObject::Request> request,
      std::shared_ptr<ras_interfaces::srv::PlaceObject::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Received place object request");

      // 1. Move to the target pose
      RCLCPP_INFO(this->get_logger(), "Moving to target pose: x=%f, y=%f, z=%f", 
                  request->target_pose.position.x,
                  request->target_pose.position.y,
                  request->target_pose.position.z);
                  
      bool move_success = Execute(request->target_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to target pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to target pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to target pose");

      // 2. Simulate gripper action directly
      RCLCPP_INFO(this->get_logger(), "Simulating gripper action: %s", 
                  request->grip_state ? "CLOSING gripper" : "OPENING gripper");
      
      // Add a short delay to simulate gripper action
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(this->get_logger(), "Gripper action completed: %s", 
                  request->grip_state ? "Gripper CLOSED" : "Gripper OPENED");
      
      // 3. Complete the operation
      response->success = true;
      response->message = "Place object operation completed successfully";
      RCLCPP_INFO(this->get_logger(), "Place object operation completed successfully");
  }

  void MoveitServer::pick_object_callback(
      const std::shared_ptr<ras_interfaces::srv::PickObject::Request> request,
      std::shared_ptr<ras_interfaces::srv::PickObject::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Received pick object request");

      // 1. Move to the target pose
      RCLCPP_INFO(this->get_logger(), "Moving to pick pose: x=%f, y=%f, z=%f", 
                  request->target_pose.position.x,
                  request->target_pose.position.y,
                  request->target_pose.position.z);
                  
      bool move_success = Execute(request->target_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to pick pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to pick pose");

      // 2. Simulate gripper action directly
      RCLCPP_INFO(this->get_logger(), "Simulating gripper action: %s", 
                  request->grip_state ? "CLOSING gripper" : "OPENING gripper");
      
      // Add a short delay to simulate gripper action
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(this->get_logger(), "Gripper action completed: %s", 
                  request->grip_state ? "Gripper CLOSED" : "Gripper OPENED");
      
      // 3. Complete the operation
      response->success = true;
      response->message = "Pick object operation completed successfully";
      RCLCPP_INFO(this->get_logger(), "Pick object operation completed successfully");
  }

  void MoveitServer::pick_front_callback(
      const std::shared_ptr<ras_interfaces::srv::PickFront::Request> request,
      std::shared_ptr<ras_interfaces::srv::PickFront::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Received pick front request");

      // target_pose.position.x = target_pose.position.x - 0.1;
      // target_pose.position.pitch = -1.57;

      // 1. Move to the target pose
      RCLCPP_INFO(this->get_logger(), "Moving to pick pose: x=%f, y=%f, z=%f", 
                  request->target_pose.position.x -= 0.12,
                  request->target_pose.position.y,
                  request->target_pose.position.z);

      // Convert current orientation quaternion to Euler angles
      double roll, pitch, yaw;
      tf2::Quaternion q(
          request->target_pose.orientation.x,
          request->target_pose.orientation.y,
          request->target_pose.orientation.z,
          request->target_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      // Modify pitch
      pitch -= 1.57;  // Subtract 1.57 radians (90 degrees)

      // Convert back to quaternion
      tf2::Quaternion q_new;
      q_new.setRPY(roll, pitch, yaw);
      request->target_pose.orientation.x = q_new.x();
      request->target_pose.orientation.y = q_new.y();
      request->target_pose.orientation.z = q_new.z();
      request->target_pose.orientation.w = q_new.w();

      bool move_success = Execute(request->target_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to pick pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to pick pose");

      // Step 3: Move to the next pose with z -= 0.1
      geometry_msgs::msg::Pose lowered_pose = request->target_pose;
      lowered_pose.position.x += 0.045;

      RCLCPP_INFO(this->get_logger(), "Moving to lowered pose: x=%f, y=%f, z=%f", 
                  lowered_pose.position.x,
                  lowered_pose.position.y,
                  lowered_pose.position.z);

      move_success = Execute(lowered_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to lowered pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to lowered pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to lowered pose");

      // 2. Simulate gripper action directly
      RCLCPP_INFO(this->get_logger(), "Simulating gripper action: %s", 
                  request->grip_state ? "CLOSING gripper" : "OPENING gripper");
      
      // Add a short delay to simulate gripper action
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(this->get_logger(), "Gripper action completed: %s", 
                  request->grip_state ? "Gripper CLOSED" : "Gripper OPENED");
      
      // 3. Complete the operation
      response->success = true;
      response->message = "Pick front operation completed successfully";

      // Step 3: Move to the next pose with z -= 0.1
      geometry_msgs::msg::Pose safe_pose = request->target_pose;
      safe_pose.position.x -= 0.05;
      // safe_pose.position.z += 0.05;

      RCLCPP_INFO(this->get_logger(), "Moving to safe pose: x=%f, y=%f, z=%f", 
                  safe_pose.position.x,
                  safe_pose.position.y,
                  safe_pose.position.z);

      move_success = Execute(safe_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to safe pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to safe pose");
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Pick front operation completed successfully");
  }

  void MoveitServer::pick_right_callback(
      const std::shared_ptr<ras_interfaces::srv::PickRight::Request> request,
      std::shared_ptr<ras_interfaces::srv::PickRight::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Received pick right request");

      // target_pose.position.x = target_pose.position.x - 0.1;
      // target_pose.position.pitch = -1.57;

      // 1. Move to the target pose
      RCLCPP_INFO(this->get_logger(), "Moving to pick pose: x=%f, y=%f, z=%f", 
                  request->target_pose.position.x,
                  request->target_pose.position.y += 0.12,
                  request->target_pose.position.z);

      // Convert current orientation quaternion to Euler angles
      double roll, pitch, yaw;
      tf2::Quaternion q(
          request->target_pose.orientation.x,
          request->target_pose.orientation.y,
          request->target_pose.orientation.z,
          request->target_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      // Modify pitch
      roll -= 1.57;  // Subtract 1.57 radians (90 degrees)
      pitch += 1.57;
      // Convert back to quaternion
      tf2::Quaternion q_new;
      q_new.setRPY(roll, pitch, yaw);
      request->target_pose.orientation.x = q_new.x();
      request->target_pose.orientation.y = q_new.y();
      request->target_pose.orientation.z = q_new.z();
      request->target_pose.orientation.w = q_new.w();

      bool move_success = Execute(request->target_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to pick pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to pick pose");

      // Step 3: Move to the next pose with z -= 0.1
      geometry_msgs::msg::Pose lowered_pose = request->target_pose;
      lowered_pose.position.y -= 0.045;

      RCLCPP_INFO(this->get_logger(), "Moving to lowered pose: x=%f, y=%f, z=%f", 
                  lowered_pose.position.x,
                  lowered_pose.position.y,
                  lowered_pose.position.z);

      move_success = Execute(lowered_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to lowered pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to lowered pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to lowered pose");

      // 2. Simulate gripper action directly
      RCLCPP_INFO(this->get_logger(), "Simulating gripper action: %s", 
                  request->grip_state ? "CLOSING gripper" : "OPENING gripper");
      
      // Add a short delay to simulate gripper action
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(this->get_logger(), "Gripper action completed: %s", 
                  request->grip_state ? "Gripper CLOSED" : "Gripper OPENED");
      
      // 3. Complete the operation
      response->success = true;
      response->message = "Pick right operation completed successfully";

      // Step 3: Move to the next pose with z -= 0.1
      geometry_msgs::msg::Pose safe_pose = request->target_pose;
      safe_pose.position.z += 0.05;
      // safe_pose.position.z += 0.05;

      RCLCPP_INFO(this->get_logger(), "Moving to safe pose: x=%f, y=%f, z=%f", 
                  safe_pose.position.x,
                  safe_pose.position.y,
                  safe_pose.position.z);

      move_success = Execute(safe_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to safe pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to safe pose");
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Pick right operation completed successfully");
  }

  void MoveitServer::pick_left_callback(
      const std::shared_ptr<ras_interfaces::srv::PickLeft::Request> request,
      std::shared_ptr<ras_interfaces::srv::PickLeft::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Received pick left request");

      // 1. Move to the target pose
      RCLCPP_INFO(this->get_logger(), "Moving to pick pose: x=%f, y=%f, z=%f", 
                  request->target_pose.position.x,
                  request->target_pose.position.y -= 0.12,
                  request->target_pose.position.z);

      // Convert current orientation quaternion to Euler angles
      double roll, pitch, yaw;
      tf2::Quaternion q(
          request->target_pose.orientation.x,
          request->target_pose.orientation.y,
          request->target_pose.orientation.z,
          request->target_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      // Modify pitch
      roll -= 4.71;  // Add 1.57 radians (90 degrees) for left
      pitch -= 1.57;
      // Convert back to quaternion
      tf2::Quaternion q_new;
      q_new.setRPY(roll, pitch, yaw);
      request->target_pose.orientation.x = q_new.x();
      request->target_pose.orientation.y = q_new.y();
      request->target_pose.orientation.z = q_new.z();
      request->target_pose.orientation.w = q_new.w();

      bool move_success = Execute(request->target_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to pick pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to pick pose");

      // Step 3: Move to the next pose with y -= 0.03
      geometry_msgs::msg::Pose lowered_pose = request->target_pose;
      lowered_pose.position.y += 0.05;

      RCLCPP_INFO(this->get_logger(), "Moving to lowered pose: x=%f, y=%f, z=%f", 
                  lowered_pose.position.x,
                  lowered_pose.position.y,
                  lowered_pose.position.z);

      move_success = Execute(lowered_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to lowered pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to lowered pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to lowered pose");

      // 2. Simulate gripper action directly
      RCLCPP_INFO(this->get_logger(), "Simulating gripper action: %s", 
                  request->grip_state ? "CLOSING gripper" : "OPENING gripper");
      
      // Add a short delay to simulate gripper action
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(this->get_logger(), "Gripper action completed: %s", 
                  request->grip_state ? "Gripper CLOSED" : "Gripper OPENED");
      
      // 3. Complete the operation
      response->success = true;
      response->message = "Pick left operation completed successfully";

      // Step 3: Move to the next pose with z += 0.05
      geometry_msgs::msg::Pose safe_pose = request->target_pose;
      safe_pose.position.z += 0.045;

      RCLCPP_INFO(this->get_logger(), "Moving to safe pose: x=%f, y=%f, z=%f", 
                  safe_pose.position.x,
                  safe_pose.position.y,
                  safe_pose.position.z);

      move_success = Execute(safe_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to safe pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to safe pose");
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Pick left operation completed successfully");
  }

  void MoveitServer::pick_rear_callback(
      const std::shared_ptr<ras_interfaces::srv::PickRear::Request> request,
      std::shared_ptr<ras_interfaces::srv::PickRear::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Received pick rear request");

      // 1. Move to the target pose (approach from rear)
      RCLCPP_INFO(this->get_logger(), "Moving to pick pose: x=%f, y=%f, z=%f", 
                  request->target_pose.position.x += 0.1,
                  request->target_pose.position.y,
                  request->target_pose.position.z);

      // Convert current orientation quaternion to Euler angles
      double roll, pitch, yaw;
      tf2::Quaternion q(
          request->target_pose.orientation.x,
          request->target_pose.orientation.y,
          request->target_pose.orientation.z,
          request->target_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      // Modify yaw for rear approach (add 180 degrees)
      pitch += 1.57;
      // Convert back to quaternion
      tf2::Quaternion q_new;
      q_new.setRPY(roll, pitch, yaw);
      request->target_pose.orientation.x = q_new.x();
      request->target_pose.orientation.y = q_new.y();
      request->target_pose.orientation.z = q_new.z();
      request->target_pose.orientation.w = q_new.w();

      bool move_success = Execute(request->target_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to pick pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to pick pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to pick pose");

      // Step 3: Move to the next pose with x += 0.03
      geometry_msgs::msg::Pose lowered_pose = request->target_pose;
      lowered_pose.position.x -= 0.03;

      RCLCPP_INFO(this->get_logger(), "Moving to lowered pose: x=%f, y=%f, z=%f", 
                  lowered_pose.position.x,
                  lowered_pose.position.y,
                  lowered_pose.position.z);

      move_success = Execute(lowered_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to lowered pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to lowered pose");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Successfully moved to lowered pose");

      // 2. Simulate gripper action directly
      RCLCPP_INFO(this->get_logger(), "Simulating gripper action: %s", 
                  request->grip_state ? "CLOSING gripper" : "OPENING gripper");
      
      // Add a short delay to simulate gripper action
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(this->get_logger(), "Gripper action completed: %s", 
                  request->grip_state ? "Gripper CLOSED" : "Gripper OPENED");
      
      // 3. Complete the operation
      response->success = true;
      response->message = "Pick rear operation completed successfully";

      // Step 3: Move to the next pose with z += 0.05
      geometry_msgs::msg::Pose safe_pose = request->target_pose;
      safe_pose.position.z += 0.05;

      RCLCPP_INFO(this->get_logger(), "Moving to safe pose: x=%f, y=%f, z=%f", 
                  safe_pose.position.x,
                  safe_pose.position.y,
                  safe_pose.position.z);

      move_success = Execute(safe_pose);
      if (!move_success) {
          response->success = false;
          response->message = "Failed to move to safe pose";
          RCLCPP_ERROR(this->get_logger(), "Failed to move to safe pose");
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Pick rear operation completed successfully");
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
