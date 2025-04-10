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

#pragma once
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#define NEW_PRIMITIVE_DECL(name) class name : public PrimitiveBehavior<name>\
    { public: \
     name(const std::string& name, const BT::NodeConfig& config = {},rclcpp::Node::SharedPtr node=nullptr) :\
         PrimitiveBehavior(name,config,node){\
            this->initialize();\
         }\
        ~name(){\
            this->destroy();\
        }\
      private:
#define END_PRIMITIVE_DECL };

namespace ras_bt_framework
{
    template <typename T>
    class PrimitiveBehavior : public BT::SyncActionNode
    {
    public:
        inline PrimitiveBehavior(const std::string& name, const BT::NodeConfig& config = {},rclcpp::Node::SharedPtr node=nullptr) :
         BT::SyncActionNode(name, config),node_(node) {
            if(node_==nullptr){
                //NOT RECOMMENDED, PROVIDED FOR SPECIAL CASES ONLY
                printf("Node is null, creating a new node\n");
                node_ = rclcpp::Node::make_shared(name+"_primitive_node");
            }
            this->setPostTickFunction(std::bind(&PrimitiveBehavior::postTick,this,std::placeholders::_1,std::placeholders::_2));
         }
        
        BT::NodeStatus postTick(BT::TreeNode &node, BT::NodeStatus status){
            RCLCPP_INFO(node_->get_logger(), "%s executed with status %d",this->name().c_str(),status);
            return status;
        }

        inline ~PrimitiveBehavior(){};
        virtual void initialize() = 0;
        virtual void destroy() = 0;
        virtual BT::NodeStatus tick() override = 0 ;

        static inline BT::PortsList ProvidedPorts(){
            auto ports = T::providedPorts();
            return ports;
        };
        static BT::PortsList mergedPorts(const BT::PortsList& childPorts)
        {
            BT::PortsList ports = { };
            ports.insert(childPorts.begin(), childPorts.end());
            return ports;
        }
        protected:
        rclcpp::Node::SharedPtr node_;
    };
}