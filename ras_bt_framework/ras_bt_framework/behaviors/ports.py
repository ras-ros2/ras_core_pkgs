
from dataclasses import dataclass
from ras_common.config.loaders.common import PoseConfig
from ..behavior_template.port import PortData
import tf_transformations
@dataclass
class PortPoseCfg(PortData):
    pose: PoseConfig

    def serialize(self):
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(self.pose.roll, self.pose.pitch, self.pose.yaw)
        return ",".join(map(str,[
            self.pose.x,
            self.pose.y,
            self.pose.z,
            qx, qy, qz, qw
        ]))
