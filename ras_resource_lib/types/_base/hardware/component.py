from dataclasses import dataclass
from .._elements.component import OssComponent
from launch.actions import ExecuteProcess
from typing import List
from ..hardware.config import HardwareConfig

@dataclass
class HardwareComponent(OssComponent):
    _internal_config: HardwareConfig
    
    def __post_init__(self):
        super().__post_init__()
        self.launch_actions = self._internal_config.launch_actions
    
    def start(self):
        return self._internal_config.start()

    def stop(self):
        return self._internal_config.stop()