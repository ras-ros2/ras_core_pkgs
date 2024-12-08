from .component import TrackableComponent
from .config import TrackableConfig

class SimModelLoader(object):
    def __init__(self,config:TrackableConfig) -> None:
        self.config=config

    def load(self) -> TrackableComponent:
        return TrackableComponent(self.config)