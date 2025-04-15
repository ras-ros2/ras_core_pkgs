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

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  };

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }
};

struct Pose2D
{
  double x, y, theta;
};

BT_JSON_CONVERTER(Pose2D, pose)
{
  add_field("x", &pose.x);
  add_field("y", &pose.y);
  add_field("theta", &pose.theta);
}

namespace BT
{
template <>
inline Pose2D convertFromString(StringView key)
{
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3)
  {
    throw BT::RuntimeError("invalid input)");
  }
  else
  {
    Pose2D output;
    output.x = convertFromString<double>(parts[0]);
    output.y = convertFromString<double>(parts[1]);
    output.theta = convertFromString<double>(parts[2]);
    return output;
  }
}
}


class MoveSimulatedManipulatorAction : public BT::StatefulActionNode
{
public:
  MoveSimulatedManipulatorAction(const std::string& name, const BT::NodeConfig& config)
    : StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<Pose2D>("goal") };
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  Pose2D _goal;
  std::chrono::system_clock::time_point _completion_time;
};

BT::NodeStatus MoveSimulatedManipulatorAction::onStart()
{
  if (!getInput<Pose2D>("goal", _goal))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  printf("[ MoveManipulator: SEND REQUEST ]. goal: x=%.1f y=%.1f theta=%.1f\n", _goal.x, _goal.y,
         _goal.theta);

  _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(220);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveSimulatedManipulatorAction::onRunning()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (std::chrono::system_clock::now() >= _completion_time)
  {
    std::cout << "[ MoveManipulator: FINISHED ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void MoveSimulatedManipulatorAction::onHalted()
{
  printf("[ MoveManipulator: ABORTED ]");
}


static const char* xml_text = R"(
<root BTCPP_format="4">

    <BehaviorTree ID="MainTree">
        <Sequence>
            <Script code=" move_goal='1;2;3' " />
            <SubTree ID="MoveManipulator" target="{move_goal}" result="{move_result}" />
            <SaySomething message="{move_result}"/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="MoveManipulator">
        <Fallback>
            <Sequence>
                <MoveSimulatedManipulator  goal="{target}"/>
                <Script code=" result:='goal reached' " />
            </Sequence>
            <ForceFailure>
                <Script code=" result:='error' " />
            </ForceFailure>
        </Fallback>
    </BehaviorTree>

</root>
 )";




int main()
{
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<MoveSimulatedManipulatorAction>("MoveSimulatedManipulator");

  factory.registerBehaviorTreeFromText(xml_text);
  auto tree = factory.createTree("MainTree");

  tree.tickWhileRunning();

  std::cout << "\n------ First BB ------" << std::endl;
  tree.subtrees[0]->blackboard->debugMessage();
  std::cout << "\n------ Second BB------" << std::endl;
  tree.subtrees[1]->blackboard->debugMessage();

  return 0;
}

