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

#include "behaviortree_cpp/bt_factory.h"
#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ras_bt_framework/primitives/say_something.hpp"
#include "ras_bt_framework/primitives/think_something_to_say.hpp"
#include "ras_bt_framework/primitives/move_to_pose.hpp"
#include "ras_bt_framework/primitives/trigger.hpp"
#include "ras_bt_framework/primitives/execute_traj.hpp"
#include "ras_bt_framework/primitives/rotate_eff.hpp"
#include "ras_bt_framework/PrimitiveActionClient.hpp"
#include "ras_bt_framework/primitives/logger_client_trigger.hpp"
#include "ras_bt_framework/primitives/move_to_joint_state.hpp"

#define REGISTER_NODE_TYPE_ROS(namespace,nodetype,rclnode) { factory->registerNodeType<namespace::nodetype>(#nodetype,rclnode); }
#define REGISTER_NODE_TYPE(namespace,nodetype) { REGISTER_NODE_TYPE_ROS(namespace,nodetype,node); }
#define REGISTER_NODE_TYPE_PRIMITIVE(nodetype) { REGISTER_NODE_TYPE(ras_bt_framework,nodetype); }

namespace ras_bt_framework {
    void registerNodes(std::shared_ptr<BT::BehaviorTreeFactory>& factory, rclcpp::Node::SharedPtr node) {
        REGISTER_NODE_TYPE_PRIMITIVE(SaySomething);
        REGISTER_NODE_TYPE_PRIMITIVE(ThinkSomethingToSay);
        REGISTER_NODE_TYPE_PRIMITIVE(MoveToPose);
        REGISTER_NODE_TYPE_PRIMITIVE(Trigger);
        REGISTER_NODE_TYPE_PRIMITIVE(ExecuteTrajectory);
        REGISTER_NODE_TYPE_PRIMITIVE(RotateEffector);
        REGISTER_NODE_TYPE_PRIMITIVE(PrimitiveActionClient);
        REGISTER_NODE_TYPE_PRIMITIVE(LoggerClientTrigger);
        REGISTER_NODE_TYPE_PRIMITIVE(MoveToJointState);
    }
}