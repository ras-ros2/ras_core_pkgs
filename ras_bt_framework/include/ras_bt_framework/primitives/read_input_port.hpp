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

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "ras_interfaces/srv/read_black.hpp"

namespace ras_bt_framework
{


NEW_PRIMITIVE_DECL(ReadInputPort)
    public:
    void initialize() override
  {
    blackboard_client = node_->create_client<ras_interfaces:srv:ReadBlack>("/read_blackboard");
  }

  void destroy() override
    {
    }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("blackboard") };
  }

  virtual BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("blackboard");

    auto request = std::make_shared<ras_interfaces::srv::ReadBlack::Request>();

    request->blackboard = msg;

    auto result_future = blackboard_client->async_send_request(
        request, std::bind(&ReadInputPort::blackboard_response, this,
                            std::placeholders::_1));  

    return BT::NodeStatus::SUCCESS;
  }
   
    void blackboard_response(rclcpp::Client<ras_interfaces::srv::ReadBlack>::SharedFuture future) {}

private:
    rclcpp::Client<ras_interfaces::srv::ReadBlack>::SharedPtr blackboard_client;
END_PRIMITIVE_DECL
};
    
