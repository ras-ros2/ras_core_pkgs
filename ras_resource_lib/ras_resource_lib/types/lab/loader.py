

from .component import LabComponent
from .config import LabCfg

class LabLoader(object):
    def __init__(self,config:LabCfg) -> None:
        self.config=config

    def load(self) -> LabComponent:
        return LabComponent(self.config)