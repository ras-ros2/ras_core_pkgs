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

#include "ras_bt_framework/BtExecutor.hpp"

namespace ras_bt_framework
{
    BTExecutor::BTExecutor(std::shared_ptr<BT::BehaviorTreeFactory> bt_factory, const rclcpp::NodeOptions & options) :
     rclcpp::Node("bt_executor", options),bt_factory_(bt_factory),bt_session_(nullptr)
    {
        this->action_server_ = rclcpp_action::create_server<BTExecutor::BTInterface>(
            this,
            "bt_executor",
            std::bind(&BTExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BTExecutor::handle_cancel, this, std::placeholders::_1),
            std::bind(&BTExecutor::handle_accepted, this, std::placeholders::_1));
        
        this->tick_srv_ = this->create_service<TickSrv_t>("/bt_tick", std::bind(&BTExecutor::bt_tick_handler,\
             this, std::placeholders::_1, std::placeholders::_2));
    }

    rclcpp_action::GoalResponse BTExecutor::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const BTExecutor::BTInterface::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with id %d", goal->bt_path);
        (void)uuid;
        (void)goal;
        if(bt_session_!=nullptr){
            abort_bt_goal();
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse BTExecutor::handle_cancel(const BTIGoalHandlePtr goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void BTExecutor::handle_accepted(const BTIGoalHandlePtr goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        std::thread{std::bind(&BTExecutor::bt_goal_handler , this, std::placeholders::_1), goal_handle}.detach();
    }
    inline static ras_interfaces::msg::BTNodeStatus cast_status(BT::NodeStatus status){
        ras_interfaces::msg::BTNodeStatus ret_status;
        switch(status){
            case BT::NodeStatus::IDLE:
                ret_status.data = ras_interfaces::msg::BTNodeStatus::IDLE;
                break;
            case BT::NodeStatus::RUNNING:
                ret_status.data = ras_interfaces::msg::BTNodeStatus::RUNNING;
                break;
            case BT::NodeStatus::SUCCESS:
                ret_status.data = ras_interfaces::msg::BTNodeStatus::SUCCESS;
                break;
            case BT::NodeStatus::FAILURE:
                ret_status.data = ras_interfaces::msg::BTNodeStatus::FAILURE;
                break;
            case BT::NodeStatus::SKIPPED:
                ret_status.data = ras_interfaces::msg::BTNodeStatus::SKIPPED;
                break;
            default:
                ret_status.data = ras_interfaces::msg::BTNodeStatus::IDLE;
        }
        return ret_status;
    }

    BTExecutor::BTSession::BTSession(const std::string& _bt_path, BTIGoalHandlePtr _goal_handle) :
        bt_path(_bt_path),goal_handle(_goal_handle),first(true) {};

    void BTExecutor::bt_goal_handler(const BTIGoalHandlePtr goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto feedback = std::make_shared<BTExecutor::BTInterface::Feedback>();
        auto result = std::make_shared<BTExecutor::BTInterface::Result>();
        
        const std::string& bt_path = goal_handle->get_goal()->bt_path;
        bt_factory_->clearRegisteredBehaviorTrees();
        bt_factory_->registerBehaviorTreeFromFile(bt_path);
        if(bt_session_!=nullptr){
            abort_bt_goal();
        }
        bt_session_ = std::make_shared<BTSession>(bt_path,goal_handle);
        bt_session_->tree = bt_factory_->createTree("MainTree");
        auto _delay = std::chrono::milliseconds(500);
        double rate = 1.0l/std::chrono::duration_cast<std::chrono::seconds>(_delay).count();
        rclcpp::Rate loop_rate(rate);
        bool loop_ok = true;

        while (rclcpp::ok() && loop_ok) {
            if((bt_session_==nullptr) || (goal_handle->is_canceling())){
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                bt_session_ = nullptr;
                return;
            }
            switch (bt_session_->status)
            {
            case BT::NodeStatus::FAILURE:
                loop_ok = false;
                break;
            case BT::NodeStatus::SUCCESS:
                loop_ok = false;
                break;
            default:
                break;
            }
            loop_rate.sleep();
            // rclcpp::spin_some(this->get_node_base_interface());
        }
        if((rclcpp::ok()) && (bt_session_->status == BT::NodeStatus::SUCCESS)) {
            result->status = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            abort_bt_goal();
        }
        bt_session_ = nullptr;

    }

    void BTExecutor::abort_bt_goal() {
        if (bt_session_ == nullptr) return;
        auto result = std::make_shared<BTExecutor::BTInterface::Result>();
        result->status = false;
        if (bt_session_->goal_handle->is_active()){
            bt_session_->goal_handle->abort(result);
        }
        RCLCPP_INFO(this->get_logger(), "Goal aborted");
        // bt_session_ = nullptr;
    }

    void BTExecutor::bt_tick_handler(const TickSrv_t::Request::SharedPtr request, TickSrv_t::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Received tick request");
        if((bt_session_==nullptr)|| (bt_session_->goal_handle->is_canceling())){
            response->status.data = ras_interfaces::msg::BTNodeStatus::SKIPPED;
            response->current_stack = request->target_stack;
            return;
        }
        auto feedback = std::make_shared<BTExecutor::BTInterface::Feedback>();
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        switch(request->exec_mode){}; //TODO: Implement this for dynamic task execution
        RCLCPP_INFO(this->get_logger(), "Executing tick");
        status = bt_session_->tree.tickExactlyOnce();
        // if(bt_session_->first){
        //     bt_session_->first = false;
        //     status = BT::NodeStatus::RUNNING;
        //     RCLCPP_INFO(this->get_logger(), "Force Status: %d", status);
        // }
        RCLCPP_INFO(this->get_logger(), "preStatus: %d", status);
        feedback->status = cast_status(status);
        response->status = feedback->status;
        response->current_stack = request->target_stack;
        RCLCPP_INFO(this->get_logger(), "Status: %d", feedback->status.data);
        bt_session_->goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Tick request processed");
    }

}