from dataclasses import dataclass
from abc import ABC,abstractmethod

@dataclass
class AssetConfig(ABC):
    label: str