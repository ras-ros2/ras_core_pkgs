from ras_bt_framework.behavior_template.instruction import PrimitiveInstruction
from dataclasses import dataclass


@dataclass
class trigger(PrimitiveInstruction):
	i_trigger : str
	o_status : str

@dataclass
class move2pose(PrimitiveInstruction):
	i_pose : str
	o_status : str

@dataclass
class Move(PrimitiveInstruction):
	i_pose : str
	o_status : str

@dataclass
class Pick(PrimitiveInstruction):
	i_pose : str
	i_trigger: str
	o_status : str

@dataclass
class Place(PrimitiveInstruction):
	i_pose : str
	i_trigger: str
	o_status : str