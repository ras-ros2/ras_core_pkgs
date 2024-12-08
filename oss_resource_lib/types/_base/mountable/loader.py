from .component import MountableComponent
from .config import MountableConfig

class MountableLoader(object):
    def __init__(self,config:MountableConfig) -> None:
        self.config=config

    def load(self) -> MountableComponent:
        return MountableComponent(self.config)