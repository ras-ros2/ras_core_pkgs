
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from dataclasses import dataclass,field
from typing import Callable,ClassVar,Set,Dict
from ras_interfaces.action import  BTPrimitive
import json
from ..behavior_template.instruction import FunctionalInstructionBase,PrimitiveInstruction

@dataclass
class PrimitiveActionClient(PrimitiveInstruction):
    name: str = field(default=None,init=False)
    input_port_names: ClassVar[Set[str]] = {"json_param","identifier"}

class PrimitiveActionManager():
    def __init__(self,node:Node):
        self._node = node
        self._action_server = ActionServer(
            self._node,
            BTPrimitive,
            'bt_primitive',
            self.execute_callback)
        self.handlers : Dict[str,type[FunctionalInstructionBase]] = dict()
    
    def register_action(self,instruction:type[FunctionalInstructionBase]):
        if not isinstance(instruction,FunctionalInstructionBase):
            raise ValueError(f"Instruction {instruction} is not a functional instruction.")
        identifier = instruction.__class__.__name__
        self.handlers[identifier] = type(instruction)
        return identifier

    def execute_callback(self, action_goal: ServerGoalHandle):
        self._node.get_logger().info('Executing goal...')
        req : BTPrimitive.Goal= action_goal.request
        feedback_msg = BTPrimitive.Feedback()
        feedback_msg.status = f"Executing action {req.identifier}"
        action_goal.publish_feedback(feedback_msg)
        handler_id = req.identifier
        if handler_id not in self.handlers:
            raise ValueError(f"Handler {handler_id} not found")
        params = json.loads(req.param_json)
        result = BTPrimitive.Result()
        result.success = False
        instruction_type = self.handlers[handler_id]
        instruction = instruction_type(**params)
        instruction.call()
        action_goal.succeed()
        self._node.get_logger().info('Goal Reached!')
        result.success = True
        return result

    def get_primitive_from(self,instruction:FunctionalInstructionBase) -> PrimitiveInstruction:
        gen_instruction = instruction
        if not isinstance(gen_instruction,FunctionalInstructionBase):
            raise ValueError(f"Instruction {instruction} is not a functional instruction.")
        identifier = self.register_action(gen_instruction)
        json_param = json.dumps(gen_instruction.params)
        return PrimitiveActionClient(input_ports={"json_param":json_param,"identifier":identifier})
    
    
    