from .component import OssComponent
from .config import AssetConfig

class AssetLoader(object):
    def __init__(self,config:AssetConfig) -> None:
        self.config=config

    def load(self) -> OssComponent:
        return OssComponent(self.config)