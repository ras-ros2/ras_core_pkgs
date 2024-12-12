from dataclasses import dataclass, field
from abc import ABC,abstractmethod
from .._base.sim_model.config import SimModelConfig
from .._base._elements.config import AssetConfig
from geometry_msgs.msg import Pose,Vector3
from typing import Tuple

@dataclass
class WorkspaceConfig(object):
    center: Vector3
    dimension: Tuple[float,float]

@dataclass
class LabCfg(SimModelConfig):
    default_robot_pose: Pose = field(init=False,default=None)
    workspace: Tuple[Vector3] = field(init=False,default=None)