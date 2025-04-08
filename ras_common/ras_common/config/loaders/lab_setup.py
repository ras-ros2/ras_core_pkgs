# 




import os
from pathlib import Path
from ..file_utils.formats.yaml import YamlFormat
import tf_transformations

from geometry_msgs.msg import Pose
from .ConfigLoaderBase import ConfigLoaderBase
from dataclasses import dataclass, field
from .common import PoseConfig
from ...globals import RAS_CONFIGS_PATH, RAS_APP_NAME
from typing import Dict, Tuple

CONFIG_FILE = Path(RAS_CONFIGS_PATH) / "lab_setup.yaml"

@dataclass
class Constraints:
    """Defines workspace and orientation constraints for a robot."""
    workspace: Dict[str, Tuple[float, float]]
    orientation: Dict[str, Tuple[float, float]]

@dataclass
class RobotConfig(ConfigLoaderBase):
    """Configuration class for a robot, including pose, home joint state, and constraints."""
    name: str
    pose: PoseConfig
    home_joint_state: Dict[str, float | int]
    constraints: Constraints = field(default_factory=lambda: Constraints(
        workspace={"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)},
        orientation={"roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)}
    ))

    @classmethod
    def from_dict(cls, data):
        """Creates a RobotConfig instance from a dictionary."""
        constraints_data = data.get("constraints", {})
        constraints = Constraints(
            workspace={k: tuple(v) for k, v in constraints_data.get("workspace", {}).items()},
            orientation={k: tuple(v) for k, v in constraints_data.get("orientation", {}).items()}
        )
        return cls(
            name=data["name"],
            pose=PoseConfig.from_dict(data["pose"]),
            home_joint_state=data["home_joint_state"],
            constraints=constraints
        )

@dataclass
class LabSetupConfig(ConfigLoaderBase):
    """Configuration class for the entire lab setup."""
    lab_name: str
    real_sim: str
    robot_name: str
    robot_config_file: str

    @classmethod
    def from_dict(cls, data):
        """Creates a LabSetupConfig instance from a dictionary."""
        return cls(
            lab_name=data["lab_name"],
            real_sim=data["real_sim"],
            robot_name=data["robot_name"],
            robot_config_file=data["robot_config_file"]
        )

class LabSetup(object):
    """Singleton class for managing lab setup configuration and robot initialization."""
    _initialized = False
    conf: LabSetupConfig = None
    robot_name: str = None
    lab_name: str = None
    robot_pose: Pose = None
    constraints: Constraints = None
    robot_config: RobotConfig = None

    @classmethod
    def init(cls):
        """Initializes the lab setup by loading configuration from a YAML file."""
        if cls._initialized:
            return
        
        yaml_obj = YamlFormat.load(CONFIG_FILE)["lab_setup"]
        cls.conf = LabSetupConfig.from_dict(yaml_obj)
        cls.lab_name = cls.conf.lab_name

        if RAS_APP_NAME == "robot":
            cls.lab_name = cls.conf.real_sim

        # Determine active robot
        cls.robot_name = cls.conf.robot_name

        # Dynamically construct robot config file path
        robot_config_path = Path(RAS_CONFIGS_PATH) / cls.conf.robot_config_file.format(robot_name=cls.robot_name)

        # Check if robot configuration file exists
        if not robot_config_path.exists():
            raise FileNotFoundError(f"Robot config file '{robot_config_path}' not found.")

        # Load robot configuration from file
        robot_data = YamlFormat.load(robot_config_path)
        cls.robot_config = RobotConfig.from_dict(robot_data)

        # Set up robot pose
        cls.robot_pose = Pose()
        r_pose = cls.robot_config.pose
        cls.robot_pose.position.x = r_pose.x
        cls.robot_pose.position.y = r_pose.y
        cls.robot_pose.position.z = r_pose.z
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(r_pose.roll, r_pose.pitch, r_pose.yaw)
        cls.robot_pose.orientation.x = qx
        cls.robot_pose.orientation.y = qy
        cls.robot_pose.orientation.z = qz
        cls.robot_pose.orientation.w = qw

        # Assign constraints
        cls.constraints = cls.robot_config.constraints

        cls._initialized = True
