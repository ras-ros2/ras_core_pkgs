from .component import ManipulatorComponent
from .config import ManipulatorCfg

class ManipulatorLoader(object):
    def __init__(self,config:ManipulatorCfg) -> None:
        self.config=config

    def load(self) -> ManipulatorComponent:
        return ManipulatorComponent(self.config)