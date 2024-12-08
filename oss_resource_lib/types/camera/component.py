from dataclasses import dataclass
from .._base._elements.component import OssComponent
from .._base.hardware.component import HardwareComponent
from .._base.sim_model.component import SimModelComponent
from .._base.mountable.component import MountableComponent
from .config import CameraCfg


@dataclass
class CameraComponent(MountableComponent,SimModelComponent,HardwareComponent):
    _internal_config: CameraCfg

    def __post_init__(self):
        self.camera_specs = self._internal_config.camera_config
        self.image_topic = self._internal_config.image_topic

