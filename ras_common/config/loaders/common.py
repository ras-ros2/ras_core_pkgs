from .ConfigLoaderBase import ConfigLoaderBase
from dataclasses import dataclass

@dataclass
class PoseConfig(ConfigLoaderBase):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float