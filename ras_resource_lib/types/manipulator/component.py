from dataclasses import dataclass
from .._base._elements.component import OssComponent
from .._base.hardware.component import HardwareComponent
from .._base.mountable.component import MountableComponent
from .._base.sim_model.component import SimModelComponent
from .config import ManipulatorCfg
from trajectory_msgs.msg import JointTrajectory

@dataclass
class ManipulatorComponent(OssComponent):
    _internal_config: ManipulatorCfg

    def __post_init__(self):
        super().__post_init__()
        self.movegroup_name = self._internal_config.movegroup_name
        self.moveit_config = self._internal_config.moveit_config
        self.launch_actions = self._internal_config.launch_actions

    def execute_trajectory(self,trajectory: JointTrajectory):
        self._internal_config.execute_trajectory(trajectory)