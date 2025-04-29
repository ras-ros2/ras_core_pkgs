
from ..behavior_template.instruction import PrimitiveInstruction
from ..behavior_template.port import PortData
from dataclasses import dataclass
from typing import ClassVar,Set
from geometry_msgs.msg import Pose

@dataclass
class PortPose(PortData):
    value: Pose

    def serialize(self):
        return f"{self.value.position.x},{self.value.position.y},{self.value.position.z},\
            {self.value.orientation.x},{self.value.orientation.y},{self.value.orientation.z},\
                {self.value.orientation.w}"



@dataclass
class ActionInstruction(PrimitiveInstruction):
    pass

@dataclass
class SaySomething(PrimitiveInstruction):
    i_message: str

@dataclass
class ThinkSomethingToSay(PrimitiveInstruction):
    i_reference: str
    o_message: str

@dataclass
class MoveToPose(PrimitiveInstruction):
    i_pose: PortPose

@dataclass
class RotateEffector(PrimitiveInstruction):
    i_rotation_angle: float

@dataclass
class Trigger(PrimitiveInstruction):
    i_trigger: bool

@dataclass
class ExecuteTrajectory(PrimitiveInstruction):
    i_sequence: int

@dataclass
class LoggerClientTrigger(PrimitiveInstruction):
    pass

@dataclass
class MoveToJointState(PrimitiveInstruction):
    i_joint_state: str
    
