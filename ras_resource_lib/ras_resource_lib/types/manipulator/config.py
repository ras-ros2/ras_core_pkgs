
from dataclasses import dataclass
from abc import ABC,abstractmethod
from .._base.hardware.config import HardwareConfig
from .._base.sim_model.config import SimModelConfig
from .._base._elements.config import AssetConfig
from .._base.mountable.config import MountableConfig
from launch.actions import ExecuteProcess
from typing import List,Callable,Dict
from trajectory_msgs.msg import JointTrajectory

@dataclass
class ManipulatorCfg(AssetConfig):

    movegroup_name : str
    moveit_config : dict
    launch_actions : List[ExecuteProcess]

    @abstractmethod
    def execute_trajectory(self,trajectory: JointTrajectory):
        pass
