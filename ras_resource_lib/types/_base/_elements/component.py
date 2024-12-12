from dataclasses import dataclass
from .config import AssetConfig
@dataclass
class OssComponent(object):
    _internal_config: AssetConfig
    
    def __post_init__(self):
        self.label = self._internal_config.label
