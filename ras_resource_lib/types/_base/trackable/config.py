from .._elements.config import AssetConfig
from dataclasses import dataclass,field
from geometry_msgs.msg import Transform

@dataclass
class ArTagConfig(object):
    id: str
    tf: Transform

@dataclass
class TrackableConfig(AssetConfig):
    ar_tag: ArTagConfig = field(init=False,default=None)