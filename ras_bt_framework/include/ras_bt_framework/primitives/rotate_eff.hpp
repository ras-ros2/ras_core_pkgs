
#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ras_interfaces/srv/rotate_effector.hpp"
#include <iostream>
#include "rclcpp/logging.hpp" // For ROS_INFO

namespace ras_bt_framework
{

        NEW_PRIMITIVE_DECL(RotateEffector)
    public:
    void initialize() override
        {
            // node_ = rclcpp::Node::make_shared("rotate_effector_node");
            rotate_eff_client_ = node_->create_client<ras_interfaces::srv::RotateEffector>("/rotate_effector");
        }

        void destroy() override
    {
    }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<double>("rotation_angle")};
        }

        virtual BT::NodeStatus tick() override
        {
            std::cout << "RotateEffector tick()" << std::endl;

            // Retrieve input angle
            auto angle = getInput<double>("rotation_angle");
            if (!angle)
            {
                throw BT::RuntimeError("Missing required input [rotation_angle]: ", angle.error());
                // return BT::NodeStatus::FAILURE;
            }

            auto request = std::make_shared<ras_interfaces::srv::RotateEffector::Request>();
            request->rotation_angle = angle.value();

            std::cout << "Sending rotation angle: " << request->rotation_angle << std::endl;

            auto result_future = rotate_eff_client_->async_send_request(
                request, std::bind(&RotateEffector::rotate_eff_response, this, std::placeholders::_1));

            // Spin and wait for the result
            if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto result = result_future.get();
                if (result->success)
                {
                    RCLCPP_INFO(node_->get_logger(), "Rotation successful.");
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Rotation failed.");
                    return BT::NodeStatus::FAILURE;
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed.");
                return BT::NodeStatus::FAILURE;
            }
        }

        void rotate_eff_response(rclcpp::Client<ras_interfaces::srv::RotateEffector>::SharedFuture future)
        {
            // Handle the response if needed (not required here for simplicity)
        }

    private:
        rclcpp::Client<ras_interfaces::srv::RotateEffector>::SharedPtr rotate_eff_client_;
END_PRIMITIVE_DECL

} // namespace ras_bt_framework
