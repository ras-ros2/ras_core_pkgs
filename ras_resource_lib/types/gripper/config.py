from dataclasses import dataclass
from .._base._elements.config import AssetConfig
from .._base.hardware.config import HardwareConfig
from .._base.mountable.config import MountableConfig
from .._base.sim_model.config import SimModelConfig
from typing import Set,Callable
from abc import ABC,abstractmethod

@dataclass
class GripperCfg(MountableConfig,HardwareConfig,SimModelConfig):
    trigger_functions: Set[Callable]
