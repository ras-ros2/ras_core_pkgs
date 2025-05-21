# Guide: Creating a New Primitive from Scratch

This guide explains how to create a new primitive in the **RAS Behavior Tree Framework**, using `PickObject` as a reference. A primitive is an atomic behavior block used within behavior trees.

---

## Overview

A primitive includes:
- service definition
- C++ behavior node
- Python wrapper
- Mappings and registration
- server-side implementation

---

## 🧾 Step 1: Define the Service Interface

Create a new service file in `ras_interfaces/srv/YourPrimitive.srv`:

```srv
# Request
geometry_msgs/Pose target_pose
bool some_parameter

---
# Response
bool success
string message
```

---

## Step 2: Create the C++ Header File

File: `ras_bt_framework/primitives/your_primitive.hpp`

```cpp
#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ras_interfaces/srv/your_primitive.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace ras_bt_framework
{
    class YourPrimitive : public BT::SyncActionNode
    {
    public:
        YourPrimitive(const std::string& name, const BT::NodeConfig& config = {}, rclcpp::Node::SharedPtr node=nullptr)
            : BT::SyncActionNode(name, config), node_(node)
        {
            if (node_ == nullptr)
            {
                node_ = rclcpp::Node::make_shared(name + "_primitive_node");
            }
            this->setPostTickFunction(std::bind(&YourPrimitive::postTick, this, std::placeholders::_1, std::placeholders::_2));
            initialize();
        }

        ~YourPrimitive()
        {
            destroy();
        }

        BT::NodeStatus postTick(BT::TreeNode& node, BT::NodeStatus status)
        {
            RCLCPP_INFO(node_->get_logger(), "%s executed with status %d", this->name().c_str(), status);
            return status;
        }

        void initialize()
        {
            RCLCPP_INFO(node_->get_logger(), "YourPrimitive initialized");
            your_primitive_client = node_->create_client<ras_interfaces::srv::YourPrimitive>("/your_primitive");
        }

        void destroy()
        {
            // Cleanup resources if needed
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::msg::Pose>("pose", "Target pose for the action"),
                BT::InputPort<bool>("some_parameter", true, "Description of the parameter")
            };
        }

        BT::NodeStatus tick() override
        {
            auto pose_msg = getInput<geometry_msgs::msg::Pose>("pose");
            if (!pose_msg)
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing required pose input");
                return BT::NodeStatus::FAILURE;
            }

            bool some_parameter = true;
            auto param_msg = getInput<bool>("some_parameter");
            if (param_msg)
            {
                some_parameter = param_msg.value();
            }

            auto request = std::make_shared<ras_interfaces::srv::YourPrimitive::Request>();
            request->target_pose = pose_msg.value();
            request->some_parameter = some_parameter;

            auto future = your_primitive_client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send request");
                return BT::NodeStatus::FAILURE;
            }

            auto response = future.get();
            if (!response->success)
            {
                RCLCPP_ERROR(node_->get_logger(), "Operation failed: %s", response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "Operation completed successfully");
            return BT::NodeStatus::SUCCESS;
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<ras_interfaces::srv::YourPrimitive>::SharedPtr your_primitive_client;
    };
}
```

---

## Step 3: Add Python Primitive Class

File: `ras_bt_framework/ras_bt_framework/behaviors/primitives.py`

```python
@dataclass
class YourPrimitive(PrimitiveInstruction):
    """
    Description of what your primitive does.

    Attributes:
        i_pose (PortPose): Target pose for the action
        i_some_parameter (bool): Description of the parameter
    """
    i_pose: PortPose
    i_some_parameter: bool = True
```

---

## Step 4: Add Action Mapping

File: `ras_bt_framework/ras_bt_framework/config/action_mappings.py`

```python
def create_your_primitive(self, pose_name: str, some_parameter: bool = True) -> YourPrimitive:
    """Create a YourPrimitive action."""
    return YourPrimitive(i_pose=self.get_pose(pose_name), i_some_parameter=some_parameter)
```

---

## Step 5: Register Default Action

File: `ras_bt_framework/ras_bt_framework/config/default_actions.py`

```python
def register_default_actions():
    # ... existing registrations ...
    action_mapping.register_action("YourPrimitive", action_mapping.create_your_primitive)
```

---

## Step 6: Update Behavior Tree Generator

File: `ras_bt_framework/ras_bt_framework/generators/behavior_tree_generator.py`

```python
__registered_primitives: Set[type[PrimitiveInstruction]] = {
    # ... existing primitives ...
    YourPrimitive,
}
```

---

## Step 7: Implement Server Logic

File: `ras_moveit/src/moveit_server.cpp`

```cpp
// In the constructor
your_primitive_srv_ = this->create_service<ras_interfaces::srv::YourPrimitive>(
    "/your_primitive",
    std::bind(&MoveitServer::your_primitive_callback, this, std::placeholders::_1, std::placeholders::_2));

// Callback implementation
void MoveitServer::your_primitive_callback(
    const std::shared_ptr<ras_interfaces::srv::YourPrimitive::Request> request,
    std::shared_ptr<ras_interfaces::srv::YourPrimitive::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received your primitive request");

    bool move_success = Execute(request->target_pose);
    if (!move_success)
    {
        response->success = false;
        response->message = "Failed to move to target pose";
        return;
    }

    // Additional logic for your primitive here

    response->success = true;
    response->message = "Operation completed successfully";
}
```

---

## Step 8: Update CMakeLists.txt

### `ras_interfaces/CMakeLists.txt`
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/your_primitive.srv"
  # ... other services ...
)
```

---

## Usage Example

```python
from ras_bt_framework.behaviors.primitives import YourPrimitive
from ras_bt_framework.config.action_mappings import action_mapping

primitive = action_mapping.create_your_primitive(
    pose_name="target_pose",
    some_parameter=True
)

behavior_tree.add_node(primitive)
```

---

## ✅ Key Points to Remember

- Follow naming conventions and code structure
- Implement proper logging and error handling
- Register your primitive in all required modules
- Test thoroughly in simulation or on hardware
- Document parameter use and expected behavior
- Use ROS 2 service/client patterns
