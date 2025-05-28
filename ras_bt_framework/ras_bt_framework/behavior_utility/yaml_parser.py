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

def read_yaml_to_pose_dict(path, calibrated_coordinates=None):
    """
    Read YAML file and convert to pose dictionary and target sequence.
    
    Args:
        path (str): Path to the YAML file
        calibrated_coordinates (dict, optional): Dictionary with calibrated x, y coordinates
            to add to the pose values if use_calibration flag is set to True in the YAML file.
            Default is None.
    
    Returns:
        tuple: (pose_dict, target_pose) - Pose dictionary and target sequence
    """
    print(f"Reading YAML file: {path}")

    with open(path, 'r') as file:
        data = yaml.safe_load(file)
    
    if 'Poses' not in data:
        raise KeyError("The key 'Poses' is missing from the YAML file.")
    
    # Check if we should use calibration
    use_calibration = data.get('use_calibration', None)
    
    # Handle the case where 'None' is a string in YAML
    if isinstance(use_calibration, str) and use_calibration.lower() == 'none':
        use_calibration = None
    
    # If use_calibration is None or False, don't apply any calibration at all
    if use_calibration is None or use_calibration is False:
        logger.log_info("No calibration specified - using original coordinates without modification")
        print(f"\033[1;33m[INFO] No calibration specified - using original coordinates without modification\033[0m")
        # Set calibrated_coordinates to None to signal that no calibration should be applied
        calibrated_coordinates = None
    # If use_calibration is True (legacy support) or a string path, and we have calibrated coordinates
    elif calibrated_coordinates:
        if isinstance(use_calibration, bool) and use_calibration:
            # Legacy support for boolean flag
            logger.log_info(f"Applying calibration offsets: X={calibrated_coordinates['x']}cm, Y={calibrated_coordinates['y']}cm")
            print(f"\033[1;36m[CALIBRATION] Applying workspace center offsets: X={calibrated_coordinates['x']}cm, Y={calibrated_coordinates['y']}cm\033[0m")
        elif isinstance(use_calibration, str):
            # This means we're using a specific calibration file
            logger.log_info(f"Using calibration file: {use_calibration}")
            logger.log_info(f"Applying calibration offsets: X={calibrated_coordinates['x']}cm, Y={calibrated_coordinates['y']}cm")
            print(f"\033[1;36m[CALIBRATION] Using file: {use_calibration}\033[0m")
            print(f"\033[1;36m[CALIBRATION] Applying workspace center offsets: X={calibrated_coordinates['x']}cm, Y={calibrated_coordinates['y']}cm\033[0m")
    
    pose_dict = {}
    for pose_name, pose_values in data['Poses'].items():
        # Create a copy of the pose values to avoid modifying the original
        adjusted_pose = copy.deepcopy(pose_values)
        
        # Apply calibration offsets if enabled and calibrated_coordinates is not None
        if calibrated_coordinates is not None:
            # Add the calibrated x and y values to the pose coordinates
            adjusted_pose['x'] += calibrated_coordinates['x']
            adjusted_pose['y'] += calibrated_coordinates['y']
            logger.log_info(f"Adjusted pose '{pose_name}': Original X={pose_values['x']}cm, Y={pose_values['y']}cm ->"  
                           f"Adjusted X={adjusted_pose['x']}cm, Y={adjusted_pose['y']}cm")
        else:
            # For None or False, we use the original coordinates without any modification
            logger.log_info(f"Using original pose '{pose_name}' without modification: X={adjusted_pose['x']}cm, Y={adjusted_pose['y']}cm")
        
        validate_pose_values(adjusted_pose)
        check_radius_constraint(pose_name, adjusted_pose)
        adjusted_pose = convert_pose_to_meters(adjusted_pose)
        # validate_pose_values(adjusted_pose)
        pose_dict[pose_name] = PortPoseCfg(pose=PoseConfig.from_dict(adjusted_pose))

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
            elif key == "Place":
                if value in pose_dict:
                    # Place is translated to move2pose + open gripper
                    target_pose.append({"move2pose": value})
                    target_pose.append({"gripper": False})
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