from .component import GripperComponent
from .config import GripperCfg

class GripperLoader(object):
    def __init__(self,config:GripperCfg) -> None:
        self.config=config

    def load(self) -> GripperComponent:
        return GripperComponent(self.config)