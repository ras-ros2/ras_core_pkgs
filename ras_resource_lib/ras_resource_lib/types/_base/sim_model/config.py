

from .._elements.config import AssetConfig
from dataclasses import dataclass

@dataclass
class SimModelConfig(AssetConfig):
    model_description: str