from dataclasses import dataclass
from .._elements.component import OssComponent
from ..mountable.config import MountableConfig

@dataclass
class MountableComponent(OssComponent):
    _internal_config: MountableConfig

    def __post_init__(self):
        super().__post_init__()
        self.base_link = self._internal_config.base_link
        self.ee_link = self._internal_config.ee_link
        self.default_origin_xyz = self._internal_config.default_origin_xyz
        self.default_origin_rpy = self._internal_config.default_origin_rpy