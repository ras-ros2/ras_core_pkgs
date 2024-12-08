from dataclasses import dataclass
from .._elements.config import AssetConfig
from typing import Iterable

@dataclass
class MountableConfig(AssetConfig):
    base_link: str
    ee_link: str
    default_origin_xyz : Iterable[float] = None
    default_origin_rpy : Iterable[float] = None