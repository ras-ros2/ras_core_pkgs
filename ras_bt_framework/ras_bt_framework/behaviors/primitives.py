"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

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
    """
    Primitive instruction to move robot to a specific joint state.
    
    Supports two modes:
    1. Full joint state: Provide values for all joints as a comma-separated string.
       Format: "joint1:value1,joint2:value2,joint3:value3,..."
       
    2. Single joint: Provide value for only one joint.
       Format: "joint_name:value"
       
    The primitive will handle moving only the specified joint while keeping others
    at their current positions when a single joint is specified.
    
    Attributes:
        i_joint_state (str): Joint state specification string
    """
    i_joint_state: str
    
