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
import yaml
from geometry_msgs.msg import Pose
import tf_transformations
CONFIGS_PATH = Path(os.environ["RAS_APP_PATH"])/"configs"
CONFIG_FILE = CONFIGS_PATH/"lab_setup.yaml"

class LabLoader(object):
    _initialized = False
    lab_name : str = None
    robot_name : str = None
    robot_pose : Pose = None
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        yaml_obj = yaml.load(CONFIG_FILE.open())["lab_setup"]
        cls.lab_name = yaml_obj["lab_name"]
        robot_obj = yaml_obj["robot"]
        cls.robot_name = robot_obj["name"]
        robot_pose_obj = robot_obj["pose"]
        cls.robot_pose = Pose()
        cls.robot_pose.position.x = robot_pose_obj["pos"]["x"]
        cls.robot_pose.position.y = robot_pose_obj["pos"]["y"]
        cls.robot_pose.position.z = robot_pose_obj["pos"]["z"]
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(robot_pose_obj["rot"]['r'], robot_pose_obj["rot"]['p'], robot_pose_obj["rot"]['y'])
        cls.robot_pose.orientation.x = qx
        cls.robot_pose.orientation.y = qy
        cls.robot_pose.orientation.z = qz
        cls.robot_pose.orientation.w = qw
        cls._initialized = True
    
    
        

