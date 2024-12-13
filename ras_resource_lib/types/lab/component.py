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
from .._base._elements.component import OssComponent
from .._base.sim_model.component import SimModelComponent
from .config import LabCfg

@dataclass
class LabComponent(SimModelComponent):
    _internal_config: LabCfg

    def __post_init__(self):
        super().__post_init__()
        self.default_robot_pose = self._internal_config.default_robot_pose
        self.workspace = self._internal_config.workspace
