

from dataclasses import dataclass
from .._base._elements.component import OssComponent
from .._base.sim_model.component import SimModelComponent
from .config import LabCfg

@dataclass
class LabComponent(SimModelComponent):
    _internal_config: LabCfg

    def __post_init__(self):
        super().__post_init__()
        self.default_robot_pose = self._internal_config.default_robot_pose
        self.workspace = self._internal_config.workspace
