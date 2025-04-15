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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ras_interfaces/action/bt_interface.hpp"
#include "ras_interfaces/srv/primitive_exec.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace ras_bt_framework
{
    
    class BTExecutor : public rclcpp::Node
    {
        public:
        using BTInterface = ras_interfaces::action::BTInterface;
        using BTIGoalHandle = rclcpp_action::ServerGoalHandle<BTInterface>;
        using BTIGoalHandlePtr = std::shared_ptr<BTIGoalHandle>;
        using TickSrv_t = ras_interfaces::srv::PrimitiveExec;

        struct BTSession
        {
            std::string bt_path;
            BT::Tree tree;
            BTIGoalHandlePtr goal_handle;
            TickSrv_t::Response::_current_stack_type current_stack;
            BT::NodeStatus status;
            bool first;

            BTSession(const std::string& _bt_path,BTIGoalHandlePtr _goal_handle);
        };

        explicit BTExecutor(std::shared_ptr<BT::BehaviorTreeFactory> bt_factory,\
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        bool load_xml(std::string file_path);

        private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,\
             std::shared_ptr<const BTInterface::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const BTIGoalHandlePtr goal_handle);
        void handle_accepted(const BTIGoalHandlePtr goal_handle);
        void bt_goal_handler(const BTIGoalHandlePtr goal_handle);
        void bt_tick_handler(const TickSrv_t::Request::SharedPtr request, TickSrv_t::Response::SharedPtr response);
        void abort_bt_goal();

        std::shared_ptr<BT::BehaviorTreeFactory> bt_factory_;
        rclcpp_action::Server<BTInterface>::SharedPtr action_server_;
        rclcpp::Service<TickSrv_t>::SharedPtr tick_srv_;
        std::shared_ptr<BTSession> bt_session_;
    };    
} // namespace ras_bt_framework
