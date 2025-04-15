"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""


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
