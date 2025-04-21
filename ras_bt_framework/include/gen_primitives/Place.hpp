// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

    NEW_PRIMITIVE_DECL(MoveAndGrip)

public:
    void initialize() override
    {}

    ~MoveAndGrip() {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("pose"),
            BT::InputPort<std::string>("trigger"),
            BT::OutputPort<std::string>("status")
        };
    }

    virtual BT::NodeStatus tick() override {
        std::cout << this->name() << " executing MoveAndGrip" << std::endl;

        // Retrieve input ports
        auto pose_res = getInput<std::string>("pose");
        auto trigger_res = getInput<std::string>("trigger");

        if (!pose_res || !trigger_res) {
            throw BT::RuntimeError("Missing required input(s)");
        }

        std::string pose = pose_res.value();
        std::string trigger = trigger_res.value();

        std::cout << "Moving to pose: " << pose << std::endl;
        std::cout << "Triggering gripper with: " << trigger << std::endl;

        // --- Replace the following mock logic with your actual calls ---
        // simulate move
        bool move_success = true; // call your motion interface
        // simulate grip
        bool grip_success = true; // call your gripper interface
        // ----------------------------------------------------------------

        if (move_success && grip_success) {
            setOutput("status", "success");
            return BT::NodeStatus::SUCCESS;
        } else {
            setOutput("status", "failure");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    // Add internal members or state tracking if needed

    END_PRIMITIVE_DECL

};
