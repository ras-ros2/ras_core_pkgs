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
from .._base.hardware.component import HardwareComponent
from .._base.mountable.component import MountableComponent
from .._base.sim_model.component import SimModelComponent
from .config import ManipulatorCfg
from trajectory_msgs.msg import JointTrajectory

@dataclass
class ManipulatorComponent(OssComponent):
    _internal_config: ManipulatorCfg

    def __post_init__(self):
        super().__post_init__()
        self.movegroup_name = self._internal_config.movegroup_name
        self.moveit_config = self._internal_config.moveit_config
        self.launch_actions = self._internal_config.launch_actions

    def execute_trajectory(self,trajectory: JointTrajectory):
        self._internal_config.execute_trajectory(trajectory)