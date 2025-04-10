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