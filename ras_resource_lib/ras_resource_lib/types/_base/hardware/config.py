

from .._elements.config import AssetConfig
from dataclasses import dataclass
from typing import List
from abc import ABC,abstractmethod
from launch.actions import ExecuteProcess


@dataclass
class HardwareConfig(AssetConfig,ABC):
    launch_actions : List[ExecuteProcess]
    
    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass