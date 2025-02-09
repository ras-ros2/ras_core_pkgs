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

import os
from pathlib import Path
from ..file_utils.formats.yaml import YamlFormat
import tf_transformations

from geometry_msgs.msg import Pose
from .ConfigLoaderBase import ConfigLoaderBase
from dataclasses import dataclass
from .common import PoseConfig
import os 
from ...globals import RAS_CONFIGS_PATH,RAS_APP_NAME
from typing import Dict
CONFIG_FILE = Path(RAS_CONFIGS_PATH)/"lab_setup.yaml"

@dataclass
class RobotConfig(ConfigLoaderBase):
    name: str
    pose: PoseConfig
    home_joint_state: Dict[str, float|int]

@dataclass
class LabSetupConfig(ConfigLoaderBase):
    lab_name: str
    real_sim: str
    robot: RobotConfig

class LabSetup(object):
    _initialized = False
    conf : LabSetupConfig = None
    robot_name : str = None
    lab_name : str = None
    robot_pose : Pose = None
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        yaml_obj = YamlFormat.load(CONFIG_FILE)["lab_setup"]
        cls.conf = LabSetupConfig.from_dict(yaml_obj)
        cls.lab_name = cls.conf.lab_name
        if (RAS_APP_NAME=="ras_robot_app"):
            cls.lab_name = cls.conf.real_sim

        cls.robot_name = cls.conf.robot.name
        cls.robot_pose = Pose()
        r_pose = cls.conf.robot.pose
        cls.robot_pose.position.x = r_pose.x
        cls.robot_pose.position.y = r_pose.y
        cls.robot_pose.position.z = r_pose.z
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(r_pose.roll, r_pose.pitch, r_pose.yaw)
        cls.robot_pose.orientation.x = qx
        cls.robot_pose.orientation.y = qy
        cls.robot_pose.orientation.z = qz
        cls.robot_pose.orientation.w = qw
        cls._initialized = True


        
    
    
        

