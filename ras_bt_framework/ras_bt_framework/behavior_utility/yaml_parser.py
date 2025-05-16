import os
import yaml
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.common import PoseConfig
from ras_common.config.loaders.lab_setup import LabSetup
import copy
import math
from ras_logging.ras_logger import RasLogger

logger = RasLogger()

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

def check_radius_constraint(pose_name, pose_values):
    """
    Checks if the pose's x and y coordinates satisfy the constraint that
    sqrt(x² + y²) < min(max_x, max_y) based on the robot's workspace constraints.
    
    Args:
        pose_name (str): Name of the pose
        pose_values (dict): Dictionary containing pose values
        
    Returns:
        bool: True if constraint is satisfied, False otherwise
    """
    # Initialize LabSetup if not already initialized
    LabSetup.init()

    # Get the robot constraints
    if LabSetup.constraints is None:
        raise ValueError(f"No constraints found for robot {LabSetup.robot_name}")

    workspace_constraints = LabSetup.constraints.workspace
    
    # Get the maximum allowable values for x and y
    x_min, x_max = workspace_constraints['x']
    y_min, y_max = workspace_constraints['y']
    
    # Get the absolute values since we're calculating radius
    max_x = min(abs(x_min), abs(x_max))
    max_y = min(abs(y_min), abs(y_max))
    
    # Use the smaller of the two as the maximum radius
    max_radius = min(max_x, max_y)
    
    x = pose_values['x']
    y = pose_values['y']
    radius = math.sqrt(x**2 + y**2)
    
    if radius >= max_radius:
        logger.log_error(f"Error for pose '{pose_name}': x or y out of bounds. " 
                    f"sqrt({x}² + {y}²) = {radius} is not less than maximum allowed radius {max_radius}")
        return False
    return True

def read_yaml_to_pose_dict(path):
    print(f"Reading YAML file: {path}")

    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'Poses' not in data:
        raise KeyError("The key 'Poses' is missing from the YAML file.")
    
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        validate_pose_values(pose_values)
        check_radius_constraint(pose_name, pose_values)
        pose_values = convert_pose_to_meters(pose_values)
        # validate_pose_values(pose_values)
        pose_dict[pose_name] = PortPoseCfg(pose=PoseConfig.from_dict(pose_values))

    if 'targets' not in data:
        raise KeyError("The key 'targets' is missing from the YAML file.")

    target_pose = []
    for action in data["targets"]:
        if isinstance(action, dict):
            key = list(action.keys())[0]
            value = action[key]

            if key in ["move2pose", "Move"]:
                if value in pose_dict:
                    target_pose.append({"move2pose": value}) 
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "gripper":
                target_pose.append({"gripper": bool(value)}) 
            elif key == "rotate":
                target_pose.append({"rotate": float(value)}) 
            elif key == "Pick":
                if value in pose_dict:
                    # Pick is translated to move2pose + close gripper
                    target_pose.append({"move2pose": value})
                    target_pose.append({"gripper": True})
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "Pick_Front":
                if value in pose_dict:

                    front_pose_1 = copy.deepcopy(pose_dict[value])
                    front_pose_1.pose.x = front_pose_1.pose.x - 0.1
                    front_pose_1.pose.pitch = -1.57
                    # Register the modified pose with a new name
                    front_pose_name_1 = f"{value}_front_1"
                    pose_dict[front_pose_name_1] = front_pose_1
                    target_pose.append({"move2pose": front_pose_name_1})

                    # Create a deep copy of the pose and modify the pitch, x, and z
                    front_pose_2 = copy.deepcopy(pose_dict[front_pose_name_1])
                    front_pose_2.pose.z = front_pose_2.pose.z - 0.08
                    # Register the modified pose with a new name
                    front_pose_name_2 = f"{value}_front_2"
                    pose_dict[front_pose_name_2] = front_pose_2
                    target_pose.append({"move2pose": front_pose_name_2})

                    front_pose_3 = copy.deepcopy(pose_dict[front_pose_name_2])
                    front_pose_3.pose.x = front_pose_3.pose.x + 0.02
                    # Register the modified pose with a new name
                    front_pose_name_3 = f"{value}_front_3"
                    pose_dict[front_pose_name_3] = front_pose_3
                    # Pick_Front is translated to move2pose with the new pose name + close gripper
                    target_pose.append({"move2pose": front_pose_name_3})
                    target_pose.append({"gripper": True})

                    front_pose_4 = copy.deepcopy(pose_dict[front_pose_name_3])
                    front_pose_4.pose.x = front_pose_4.pose.x - 0.03
                    front_pose_4.pose.z = front_pose_4.pose.z + 0.03
                    front_pose_name_4 = f"{value}_front_4"
                    pose_dict[front_pose_name_4] = front_pose_4
                    target_pose.append({"move2pose": front_pose_name_4})

                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "Pick_Right":
                if value in pose_dict:
                    # Create a sequence of poses for side approach
                    # First pose: move to the side
                    side_pose_1 = copy.deepcopy(pose_dict[value])
                    side_pose_1.pose.y = side_pose_1.pose.y + 0.12
                    side_pose_1.pose.z = side_pose_1.pose.z - 0.06
                    side_pose_1.pose.roll = 1.57  # Rotate 90 degrees to face the side
                    # Register the modified pose with a new name
                    side_pose_name_1 = f"{value}_side_1"
                    pose_dict[side_pose_name_1] = side_pose_1
                    target_pose.append({"move2pose": side_pose_name_1})

                    # Second pose: adjust position for side approach
                    side_pose_2 = copy.deepcopy(pose_dict[side_pose_name_1])
                    side_pose_2.pose.pitch = 1.57
                    # Register the modified pose with a new name
                    side_pose_name_2 = f"{value}_side_2"
                    pose_dict[side_pose_name_2] = side_pose_2
                    target_pose.append({"move2pose": side_pose_name_2})

                    # Third pose: approach the object
                    side_pose_3 = copy.deepcopy(pose_dict[side_pose_name_2])
                    side_pose_3.pose.y = side_pose_3.pose.y - 0.05
                    # Register the modified pose with a new name
                    side_pose_name_3 = f"{value}_side_3"
                    pose_dict[side_pose_name_3] = side_pose_3
                    # Pick_Right is translated to move2pose with the new pose name + close gripper
                    target_pose.append({"move2pose": side_pose_name_3})
                    target_pose.append({"gripper": True})

                    # Fourth pose: retreat with object
                    side_pose_4 = copy.deepcopy(pose_dict[side_pose_name_3])
                    side_pose_4.pose.y = side_pose_4.pose.y + 0.05
                    side_pose_4.pose.z = side_pose_4.pose.z + 0.03
                    side_pose_name_4 = f"{value}_side_4"
                    pose_dict[side_pose_name_4] = side_pose_4
                    target_pose.append({"move2pose": side_pose_name_4})
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "Place":
                if value in pose_dict:
                    # Place is translated to move2pose + open gripper
                    target_pose.append({"move2pose": value})
                    target_pose.append({"gripper": False})
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "PlaceObject":
                # New unified place action that combines move + gripper control
                if value in pose_dict:
                    # PlaceObject is a single action (not decomposed)
                    target_pose.append({"PlaceObject": value})
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "PickObject":
                # New unified pick action that combines move + gripper control
                if value in pose_dict:
                    # PickObject is a single action (not decomposed)
                    target_pose.append({"PickObject": value})
                else:
                    raise KeyError(f"Undefined pose '{value}' in 'targets' section. Available poses: {list(pose_dict.keys())}")
            elif key == "single_joint_state":
                # Format should be [joint_index, joint_value]
                if isinstance(value, list) and len(value) == 2:
                    joint_index = int(value[0])
                    joint_value = float(value[1])
                    target_pose.append({"single_joint_state": [joint_index, joint_value]})
                else:
                    raise ValueError("single_joint_state expects a list [joint_index, joint_value]")
            else:
                raise KeyError(f"Unknown target action: {key}")
        else:
            raise ValueError("Invalid format in 'targets'. Each entry must be a dictionary.")

    return pose_dict, target_pose