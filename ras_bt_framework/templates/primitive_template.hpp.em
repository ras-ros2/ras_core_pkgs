// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

    NEW_PRIMITIVE_DECL(@(class_name))
    
    public:
    void initialize() override
    {}

    void destroy() override
    {}
    
    static BT::PortsList providedPorts()
    {
        return { @(provided_ports) };
    }

   virtual BT::NodeStatus tick() override {
    std::cout << (this->name()) << std::endl;
    // add your code here

    return BT::NodeStatus::SUCCESS;
   }
   private:
   // add your members here

   END_PRIMITIVE_DECL

};
