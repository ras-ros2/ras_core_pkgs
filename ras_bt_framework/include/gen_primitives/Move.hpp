// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

    NEW_PRIMITIVE_DECL(Move2pose)
    
    public:
    void initialize() override
    {}

    ~Move2pose() {}
    
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("pose"),
 BT::OutputPort<std::string>("status") };
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
