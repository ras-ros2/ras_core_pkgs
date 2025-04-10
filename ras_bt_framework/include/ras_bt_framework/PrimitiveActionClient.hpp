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

#pragma once
#include "PrimitiveBehavior.hpp"
#include "ras_interfaces/action/bt_primitive.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace ras_bt_framework
{
    NEW_PRIMITIVE_DECL(PrimitiveActionClient) 
        public:
        void initialize() override
        {
            status_= BT::NodeStatus::RUNNING;
            auto _delay = std::chrono::milliseconds(500);
            double rate = 1.0l/std::chrono::duration_cast<std::chrono::seconds>(_delay).count();
            loop_rate_ = std::make_shared<rclcpp::Rate>(rate);
            client_ptr_ = rclcpp_action::create_client<ras_interfaces::action::BTPrimitive>(node_,"bt_primitive");
        }
        using BTPrimitive = ras_interfaces::action::BTPrimitive;
        using GoalHandle = rclcpp_action::ClientGoalHandle<BTPrimitive>;

        void destroy() override
    {
    }

        void execute(const std::string& identifier, const std::string& json_param){
            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
                status_ = BT::NodeStatus::FAILURE;
                return;
            }

            auto goal_msg = ras_interfaces::action::BTPrimitive::Goal();
            goal_msg.identifier = identifier;
            goal_msg.param_json = json_param;
            auto send_goal_options = rclcpp_action::Client<BTPrimitive>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&PrimitiveActionClient::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&PrimitiveActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
            std::bind(&PrimitiveActionClient::result_callback, this, std::placeholders::_1);
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        }

        void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            status_ = BT::NodeStatus::FAILURE;
            } else {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
            status_ = BT::NodeStatus::RUNNING;
            }
        }
          void feedback_callback( GoalHandle::SharedPtr,
            const std::shared_ptr<const BTPrimitive::Feedback> feedback)
        {
            std::stringstream ss;
            ss << "Feedback received: " << feedback->status;
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            status_ = BT::NodeStatus::RUNNING;
        }

        void result_callback(const GoalHandle::WrappedResult & result)
        {
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
                status_ = BT::NodeStatus::SUCCESS;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                status_ = BT::NodeStatus::FAILURE;
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                status_ =  BT::NodeStatus::FAILURE;
                return;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                status_ = BT::NodeStatus::FAILURE;
                return;
            }
            
        }
        
        static BT::PortsList providedPorts() {
            BT::PortsList ports = { BT::InputPort<std::string>("identifier"),BT::InputPort<std::string>("json_param") };
            return mergedPorts(ports);
        }
        virtual BT::NodeStatus tick() override {
            std::string identifier = getInput<std::string>("identifier").value();
            std::string json_param = getInput<std::string>("json_param").value();

            RCLCPP_INFO(node_->get_logger(), "Executing PrimitiveActionClient with json_param: %s",json_param.c_str());
            //    get_logger()->info();
            execute(identifier,json_param);
            
            while((rclcpp::ok())){
                // rclcpp::spin_some(node_);
                switch (status_)
                {
                case (BT::NodeStatus::RUNNING):
                    {
                        break;
                    }
                case (BT::NodeStatus::SUCCESS):
                    {
                        return BT::NodeStatus::SUCCESS;
                    }
                case (BT::NodeStatus::FAILURE):
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                
                default:
                    return status_;
                }
                loop_rate_->sleep();
            }
            return BT::NodeStatus::FAILURE;
        }
        private:
        BT::NodeStatus status_;
        rclcpp_action::Client<ras_interfaces::action::BTPrimitive>::SharedPtr client_ptr_;
        std::shared_ptr<rclcpp::Rate> loop_rate_;
    END_PRIMITIVE_DECL
} // namespace ras_bt_framework