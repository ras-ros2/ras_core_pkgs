from .component import HardwareComponent
from .config import HardwareConfig

class HardwareLoader(object):
    def __init__(self,config:HardwareConfig) -> None:
        self.config=config

    def load(self) -> HardwareComponent:
        return HardwareComponent(self.config)