from .._elements.component import OssComponent
from dataclasses import dataclass
from .config import SimModelConfig

@dataclass
class SimModelComponent(OssComponent):
    _internal_config: SimModelConfig
    def __post_init__(self):
        super().__post_init__()
        self.model_description = self._internal_config.model_description