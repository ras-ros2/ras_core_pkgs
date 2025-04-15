from ras_common.config.loaders.ConfigLoaderBase import ConfigLoaderBase
from ras_common.config.loaders.common import PoseConfig
from dataclasses import dataclass
from typing import Dict

@dataclass
class GridLocation(ConfigLoaderBase):
    x: float
    y: float

@dataclass
class GridConfig(ConfigLoaderBase):
    pose: PoseConfig
    locations: Dict[str, GridLocation]

    def __post_init__(self):
        for k,v in self.locations.items():
            self.locations[k] = GridLocation.from_dict(v)


    
    
