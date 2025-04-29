
#include "rclcpp/rclcpp.hpp"
#include "ras_bt_framework/BtFactory.hpp"
#include "ras_bt_framework/BtExecutor.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <type_traits>


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node> ("bt_executor_node");
    std::shared_ptr<BT::BehaviorTreeFactory> factory = std::make_shared<BT::BehaviorTreeFactory>();
    ras_bt_framework::registerNodes(factory,node);
    std::shared_ptr<ras_bt_framework::BTExecutor> bt_executor = std::make_shared<ras_bt_framework::BTExecutor>(factory);
    // executor.add_node(node);
    executor.add_node(bt_executor);
    while (rclcpp::ok())
    {
        executor.spin_some();
        rclcpp::spin_some(node);
    }
    
    // executor.spin();
    return 0;
}