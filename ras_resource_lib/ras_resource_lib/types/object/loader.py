

from .component import ObjectComponent
from .config import ObjectCfg

class ManipulatorLoader(object):
    def __init__(self,config:ObjectCfg) -> None:
        self.config=config

    def load(self) -> ObjectComponent:
        return ObjectComponent(self.config)