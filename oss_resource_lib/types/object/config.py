from dataclasses import dataclass,field
from abc import ABC,abstractmethod
from .._base.sim_model.config import SimModelConfig
from .._base._elements.config import AssetConfig
from .._base.trackable.config import TrackableConfig
from enum import Enum

class Axis(Enum):
    NONE = 0

    X = 1
    Y = 2
    Z = 3


@dataclass
class ObjectCfg(TrackableConfig, SimModelConfig):
    symmetrical_about_axis: Axis|None = field(init=False,default=None)
