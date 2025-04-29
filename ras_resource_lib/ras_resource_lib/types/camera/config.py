from dataclasses import dataclass
from .._base._elements.config import AssetConfig
from .._base.hardware.config import HardwareConfig
from .._base.mountable.config import MountableConfig
from .._base.sim_model.config import SimModelConfig
from typing import Tuple
from abc import ABC,abstractmethod

@dataclass
class CameraSpecs(object):
    fov_h : float
    fov_v : float
    


@dataclass
class CameraCfg(MountableConfig,HardwareConfig,SimModelConfig):
    camera_config : CameraSpecs
    image_topic : str

