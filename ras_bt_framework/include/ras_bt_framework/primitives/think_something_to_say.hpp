
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
    
