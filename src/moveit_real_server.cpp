#include "moveit_real_server.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Moveit Server Init");
static const std::string PLANNING_GROUP_ARM = "lite6";

MoveitRealServer::MoveitRealServer(std::shared_ptr<rclcpp::Node> move_group_node)
    : Node("moveit_server")
    
        {
        this->declare_parameter("move_group_name", "lite6");

        std::string move_group_name = this->get_parameter("move_group_name").as_string();

        move_group_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node,
            move_group_name
        );
        RCLCPP_INFO(LOGGER, "Real Node Started");

        execute_traj_srv = this->create_service<oss_interfaces::srv::ActionTraj>(
            "trajectory_topic",
            std::bind(&MoveitRealServer::trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));

        }
  
void MoveitRealServer::trajectory_callback(const std::shared_ptr<oss_interfaces::srv::ActionTraj::Request> request,
      std::shared_ptr<oss_interfaces::srv::ActionTraj::Response> response)
    {
    RCLCPP_INFO(this->get_logger(), "Received trajectory message");

    moveit_msgs::msg::RobotTrajectory robot_trajectory;

    // Convert JointTrajectory to RobotTrajectory
    robot_trajectory.joint_trajectory = request->traj;

    bool success = (move_group_arm->execute(robot_trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    response->success = 1;
    }


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("moveit_server", node_options);
  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<MoveitRealServer> planner_node =
      std::make_shared<MoveitRealServer>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}
