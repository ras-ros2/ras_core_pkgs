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

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(ThinkSomethingToSay)
    public:
    void initialize() override
  {}
  void destroy() override
    {
    }

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("reference");

    setOutput<std::string>("message", msg.value());
    return BT::NodeStatus::SUCCESS;
  };

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("reference"),BT::OutputPort<std::string>("message") };
  }
END_PRIMITIVE_DECL
};
    
