
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