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

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ras_interfaces/srv/pose_req.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/logging.hpp"  // For ROS_INFO
#include "ras_interfaces/srv/joint_req.hpp"


namespace BT
{
template <>
inline sensor_msgs::msg::JointState convertFromString(StringView str)
{
    // Expect a string like: "joint1:0.2,joint2:0.25,joint3:0.1,joint4:0.0,joint5:0.0,joint6:0.0"
    
    sensor_msgs::msg::JointState joint_state;
    auto parts = BT::splitString(str, ',');
    for (StringView& part : parts)
    {
        auto pair = BT::splitString(part, ':');
        if (pair.size() != 2)
        {
            throw BT::RuntimeError("Expected a string like: joint1:0.0,joint2:0.0,joint3:0.0,joint4:0.0,joint5:0.0,joint6:0.0");
        }
        for (const StringView& _str: pair)
        {
            if (_str.empty())
            {
                throw BT::RuntimeError("Expected a string like: joint1:0.0,joint2:0.0,joint3:0.0,joint4:0.0,joint5:0.0,joint6:0.0");
            }
        }
        joint_state.name.push_back(std::string(pair[0]));
        joint_state.position.push_back(convertFromString<double>(pair[1]));
    }

    return joint_state;
}
}


namespace ras_bt_framework
{
    NEW_PRIMITIVE_DECL(MoveToJointState)
        public:
        void initialize() override
        {
            // Initialize other members here, like the ROS node
            // node_ = rclcpp::Node::make_shared("move_to_joint_state_node");
            move_to_joint_state = node_->create_client<ras_interfaces::srv::JointReq>("/move_to_joint_states");
        }

        void destroy() override
        {
        }
        
        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<sensor_msgs::msg::JointState>("joint_state") };
        }
        
        virtual BT::NodeStatus tick() override {
            std::cout << ("MoveToJointState") << std::endl;

            auto msg = getInput<sensor_msgs::msg::JointState>("joint_state");

            auto request = std::make_shared<ras_interfaces::srv::JointReq::Request>();

            // Direct assignment without using 'expected'
            request->joints = msg.value();

            auto result_future = move_to_joint_state->async_send_request(
                    request, std::bind(&MoveToJointState::move_to_joint_state_response, this,
                                        std::placeholders::_1));  

            if ((rclcpp::spin_until_future_complete(node_, result_future) ==
                rclcpp::FutureReturnCode::SUCCESS)&&(result_future.get()->success))
            {
            return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        void move_to_joint_state_response(rclcpp::Client<ras_interfaces::srv::JointReq>::SharedFuture future) {
            // Handle the response if needed
        }


    private:
        rclcpp::Client<ras_interfaces::srv::JointReq>::SharedPtr move_to_joint_state;
        sensor_msgs::msg::JointState joint_state_msg_;

    END_PRIMITIVE_DECL
};

