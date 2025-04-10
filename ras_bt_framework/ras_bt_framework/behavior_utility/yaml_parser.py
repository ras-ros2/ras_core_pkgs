import os
import yaml
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.common import PoseConfig
from ras_common.config.loaders.lab_setup import LabSetup

def validate_pose_values(pose_values):
    """
    Validate pose values against robot-specific constraints defined in lab_setup.yaml.
    Uses the currently active robot's constraints from LabSetup to validate workspace and orientation limits.

    Args:
        pose_values (dict): Dictionary containing pose values with the following keys:
            - x (float): X coordinate in meters
            - y (float): Y coordinate in meters
            - z (float): Z coordinate in meters
            - roll (float): Roll angle in radians
            - pitch (float): Pitch angle in radians
            - yaw (float): Yaw angle in radians

    Raises:
        ValueError: If any of the following conditions are met:
            - No constraints found for the current robot
            - Unknown constraint key in pose_values
            - Pose value is outside the robot's workspace or orientation limits
            - Missing required keys in pose_values

    Example:
        >>> pose = {
        ...     'x': 0.5, 'y': 0.3, 'z': 0.4,
        ...     'roll': 0.0, 'pitch': 0.0, 'yaw': 1.57
        ... }
        >>> validate_pose_values(pose)  # Will raise ValueError if pose is invalid
    """
    # Initialize LabSetup if not already initialized
    LabSetup.init()

    # Get the robot constraints
    if LabSetup.constraints is None:
        raise ValueError(f"No constraints found for robot {LabSetup.robot_name}")

    workspace_constraints = LabSetup.constraints.workspace
    orientation_constraints = LabSetup.constraints.orientation

    for key, value in pose_values.items():
        if key in workspace_constraints:
            min_val, max_val = workspace_constraints[key]
        elif key in orientation_constraints:
            min_val, max_val = orientation_constraints[key]
        else:
            raise ValueError(f"Unknown constraint key: {key}")

        if not (min_val <= value <= max_val):
            raise ValueError(
                f"Invalid value for {key}: {value}. Must be between {min_val} and {max_val} for {LabSetup.robot_name} robot"
            )

def convert_pose_to_meters(pose_values):
    """
    Converts pose values from centimeters to meters.

    Args:
        pose_values (dict): A dictionary containing pose values in centimeters.

    Returns:
        dict: A dictionary with pose values converted to meters.
    """
    return {
        'x': pose_values['x'] / 100,  # Convert cm to meters
        'y': pose_values['y'] / 100,
        'z': pose_values['z'] / 100,
        'roll': pose_values['roll'],  # Assuming roll, pitch, yaw are in radians
        'pitch': pose_values['pitch'],
        'yaw': pose_values['yaw']
    }

def read_yaml_to_pose_dict(path):
    """
    Read and parse a YAML experiment file containing poses and target actions.
    Validates all poses against the robot's constraints and processes target actions.

    Args:
        path (str): Path to the YAML experiment file

    Returns:
        tuple: A tuple containing:
            - pose_dict (dict): Dictionary mapping pose names to PortPoseCfg objects
            - target_pose (list): List of processed target actions, where each action is a dictionary
                                with one of the following formats:
                                - {'move2pose': str} - Move to a named pose
                                - {'gripper': bool} - Gripper action (True=open, False=close)
                                - {'rotate': float} - Rotate by angle in radians

    Raises:
        KeyError: If any of the following are missing from the YAML:
            - 'Poses' section
            - 'targets' section
            - Referenced pose name in move2pose action
        ValueError: If any of the following conditions are met:
            - Invalid pose values (via validate_pose_values)
            - Invalid target action format
            - Unknown target action type

    Example YAML Format:
        Poses:
            home:
                x: 0.0
                y: 0.0
                z: 0.5
                roll: 0.0
                pitch: 0.0
                yaw: 0.0
            pick:
                x: 0.3
                y: 0.3
                z: 0.2
                roll: 3.14
                pitch: 0.0
                yaw: 1.57

        targets:
            - move2pose: "home"
            - gripper: true
            - move2pose: "pick"
            - gripper: false
            - rotate: 1.57
    """
    print(f"Reading YAML file: {path}")

    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'Poses' not in data:
        raise KeyError("The key 'poses' is missing from the YAML file.")
    
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        
        # Convert pose values from centimeters to meters
        pose_values = convert_pose_to_meters(pose_values)
        
        validate_pose_values(pose_values)
        pose_dict[pose_name] = PortPoseCfg(pose=PoseConfig.from_dict(pose_values))

    if 'targets' not in data:
        raise KeyError("The key 'targets' is missing from the YAML file.")

    target_pose = []
    for action in data["targets"]:
        if isinstance(action, dict):
            key = list(action.keys())[0]
            value = action[key]

            if key == "move2pose":
                if value in pose_dict:
                    target_pose.append({key: value}) 
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "gripper":
                target_pose.append({key: bool(value)}) 
            elif key == "rotate":
                target_pose.append({key: float(value)}) 
            else:
                raise KeyError(f"Unknown target action: {key}")
        else:
            raise ValueError("Invalid format in 'targets'. Each entry must be a dictionary.")

    return pose_dict, target_pose
