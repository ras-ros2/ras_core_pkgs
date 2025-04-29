

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(SaySomething)
    public:
    void initialize() override
    {
    }
    void destroy() override
    {
    }

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
    // node_->get_logger()("Robot says: {}", msg.value());
    RCLCPP_INFO(node_->get_logger(), "Robot says: %s", msg.value().c_str());
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  };

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }
END_PRIMITIVE_DECL
};