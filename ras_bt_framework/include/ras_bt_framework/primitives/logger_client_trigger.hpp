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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(LoggerClientTrigger)
    public:
    void initialize() override
  {

    // node_ = rclcpp::Node::make_shared("logger_client_trigger_node");
    trigger_client = node_->create_client<std_srvs::srv::SetBool>("/dummy_logging_server"); // TODO (Sachin): Change the service name
    trigger_client->wait_for_service(); // TODO (Sachin): Check if this is required and where it should be executed
    
  }
  void destroy() override
    {
    }

  BT::NodeStatus tick() override
  {

    std::cout << ("Logger Client Trigger") << std::endl;

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    request->data = true;
    auto result_future = trigger_client->async_send_request(
        request, std::bind(&LoggerClientTrigger::trigger_response, this,
                            std::placeholders::_1)); 

    if (rclcpp::spin_until_future_complete(node_, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
    return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
   }

    void trigger_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trigger_client;

END_PRIMITIVE_DECL

  };  