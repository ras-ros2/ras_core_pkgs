
from dataclasses import dataclass
from .._base._elements.component import OssComponent
from .._base.sim_model.component import SimModelComponent
from .._base.trackable.component import TrackableComponent
from .config import ObjectCfg

@dataclass
class ObjectComponent(TrackableComponent,SimModelComponent):
    _internal_config: ObjectCfg

    def __post_init__(self):
        super().__post_init__()
        self.symmetrical_about_axis = self._internal_config.symmetrical_about_axis
