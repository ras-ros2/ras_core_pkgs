
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