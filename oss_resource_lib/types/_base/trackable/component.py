from .._elements.component import OssComponent
from dataclasses import dataclass
from .config import TrackableConfig


@dataclass
class TrackableComponent(OssComponent):
    _internal_config: TrackableConfig
    def __post_init__(self):
        super().__post_init__()
        self.ar_tag = self._internal_config.ar_tag