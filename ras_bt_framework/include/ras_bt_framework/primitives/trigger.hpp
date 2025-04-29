
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

NEW_PRIMITIVE_DECL(Trigger)
    public:
    void initialize() override
  {

    trigger_client = node_->create_client<std_srvs::srv::SetBool>("/fake_gripper");
    
  }
  void destroy() override
    {
    }
  BT::NodeStatus tick() override
  {

    std::cout << (this->name()) << std::endl;

    auto msg = getInput<bool>("trigger");

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    if (msg == true)
    {
        request->data = true;
    }
    else
    {
        request->data = false;
    }

    auto result_future = trigger_client->async_send_request(
        request, std::bind(&Trigger::trigger_response, this,
                            std::placeholders::_1)); 

    if ((rclcpp::spin_until_future_complete(node_, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)&&result_future.get()->success)
    {
    return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
   }

    void trigger_response(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("trigger") };
  }

private:
rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trigger_client;

END_PRIMITIVE_DECL

  };  