/*
 * 
 * Copyright (C) 2024 RAS Core
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
*/

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ras_interfaces/srv/pick_object.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/logging.hpp"

namespace BT
{
// We rely on the convertFromString function for geometry_msgs::msg::Pose 
// that's already defined in move_to_pose.hpp
}

namespace ras_bt_framework
{
    // Direct class definition without using the macro to avoid template issues
    class PickObject : public BT::SyncActionNode
    {
    public:
        PickObject(const std::string& name, const BT::NodeConfig& config = {}, rclcpp::Node::SharedPtr node=nullptr) :
            BT::SyncActionNode(name, config), node_(node) 
        {
            if(node_ == nullptr) {
                //NOT RECOMMENDED, PROVIDED FOR SPECIAL CASES ONLY
                printf("Node is null, creating a new node\n");
                node_ = rclcpp::Node::make_shared(name+"_primitive_node");
            }
            this->setPostTickFunction(std::bind(&PickObject::postTick, this, std::placeholders::_1, std::placeholders::_2));
            initialize();
        }
        
        ~PickObject() {
            destroy();
        }
        
        BT::NodeStatus postTick(BT::TreeNode &node, BT::NodeStatus status) {
            RCLCPP_INFO(node_->get_logger(), "%s executed with status %d", this->name().c_str(), status);
            return status;
        }
        
        void initialize() {
            RCLCPP_INFO(node_->get_logger(), "PickObject initialized");
            pick_object_client = node_->create_client<ras_interfaces::srv::PickObject>("/pick_object");
        }

        void destroy() {
            // Cleanup resources if needed
        }
        
        static BT::PortsList providedPorts() {
            return { 
                BT::InputPort<geometry_msgs::msg::Pose>("pose", "Target pose for picking"),
                BT::InputPort<bool>("grip_state", true, "Gripper state (true for grip, false for release)")
            };
        }
        
        BT::NodeStatus tick() override {
            std::cout << (this->name()) << " executing PickObject" << std::endl;

            // 1. Get pose input
            auto pose_msg = getInput<geometry_msgs::msg::Pose>("pose");
            if (!pose_msg) {
                RCLCPP_ERROR(node_->get_logger(), "Missing required pose input");
                return BT::NodeStatus::FAILURE;
            }

            // 2. Get grip state input (default to true/close for picking)
            bool grip_state = true;
            auto grip_state_msg = getInput<bool>("grip_state");
            if (grip_state_msg) {
                grip_state = grip_state_msg.value();
            }

            // 3. Send pick object request
            auto request = std::make_shared<ras_interfaces::srv::PickObject::Request>();
            request->target_pose = pose_msg.value();
            request->grip_state = grip_state;

            auto future = pick_object_client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, future) != 
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send pick object request");
                return BT::NodeStatus::FAILURE;
            }

            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(node_->get_logger(), "Pick object operation failed: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "PickObject completed successfully");
            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<ras_interfaces::srv::PickObject>::SharedPtr pick_object_client;
    };
} 