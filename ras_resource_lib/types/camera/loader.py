from .component import CameraComponent
from .config import CameraCfg

class CameraLoader(object):
    def __init__(self,config:CameraCfg) -> None:
        self.config=config

    def load(self) -> CameraComponent:
        return CameraComponent(self.config)