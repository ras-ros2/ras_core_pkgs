

from .component import SimModelComponent
from .config import SimModelConfig

class SimModelLoader(object):
    def __init__(self,config:SimModelConfig) -> None:
        self.config=config

    def load(self) -> SimModelComponent:
        return SimModelComponent(self.config)