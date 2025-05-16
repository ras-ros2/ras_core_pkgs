import os
import yaml
from typing import Dict, List, Optional, Tuple
from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.keywords import TargetPoseMap, rotate, gripper, joint_state, Pick, Pick_Front, Place
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.lab_setup import LabSetup
from ras_common.config.loaders.objects import ObjectTypes
from .behavior_utility.yaml_parser import read_yaml_to_pose_dict
from copy import deepcopy
from ..behaviors.keywords import MoveToPose

class BatMan:
    """
    A class for managing and executing robot experiments with a focus on pick-and-place tasks.
    
    This class extends the functionality of ExperimentsService to provide specialized
    methods for handling pick-and-place operations and experiment sequences.
    
    Attributes:
        pose_map (TargetPoseMap): Map of registered poses and their configurations
        experiments_dir (str): Directory containing experiment YAML files
        current_experiment (str): Name of the currently loaded experiment
    """
    
    def __init__(self, experiments_dir: str):
        """
        Initialize the BatMan experiment manager.
        
        Args:
            experiments_dir (str): Path to the directory containing experiment YAML files
        """
        self.pose_map = TargetPoseMap()
        self.experiments_dir = experiments_dir
        self.current_experiment = None
        
    def load_experiment(self, experiment_name: str) -> Tuple[Dict[str, PortPoseCfg], List[Dict]]:
        """
        Load an experiment configuration from a YAML file.
        
        Args:
            experiment_name (str): Name of the experiment file (without .yaml extension)
            
        Returns:
            Tuple[Dict[str, PortPoseCfg], List[Dict]]: A tuple containing:
                - Dictionary mapping pose names to their configurations
                - List of target actions to execute
                
        Raises:
            FileNotFoundError: If the experiment file doesn't exist
        """
        experiment_path = os.path.join(self.experiments_dir, f"{experiment_name}.yaml")
        if not os.path.exists(experiment_path):
            raise FileNotFoundError(f"Experiment file not found: {experiment_path}")
            
        pose_dict, target_pose = read_yaml_to_pose_dict(experiment_path)
        
        # Register all poses with the pose map
        for pose_name, pose in pose_dict.items():
            self.pose_map.register_pose(pose_name, pose)
            
        self.current_experiment = experiment_name
        return pose_dict, target_pose
        
    def create_behavior_sequence(self, target_pose: List[Dict]) -> BehaviorModuleSequence:
        """
        Create a behavior sequence from a list of target actions.
        
        Args:
            target_pose (List[Dict]): List of target actions to execute
            
        Returns:
            BehaviorModuleSequence: Sequence of behavior modules to execute
            
        Raises:
            ValueError: If an unknown target action is encountered
        """
        sequence = BehaviorModuleSequence()
        
        for action in target_pose:
            if isinstance(action, dict):
                key = list(action.keys())[0]
                value = action[key]
                
                if key == "move2pose":
                    sequence.add_child(self.pose_map.move2pose_module(value))
                elif key == "Move":
                    sequence.add_child(self.pose_map.move2pose_module(value))
                elif key == "gripper":
                    sequence.add_child(gripper(value))
                elif key == "rotate":
                    sequence.add_child(rotate(value))
                elif key == "joint_state":
                    sequence.add_child(joint_state(value))
                elif key == "Pick":
                    # Composite action: move to pose and close gripper
                    if isinstance(value, dict):
                        pose_name = value.get("pose")
                    else:
                        pose_name = value

                    sequence.add_child(self.pose_map.move2pose_module(pose_name))
                    sequence.add_child(gripper("close"))
                elif key == "Pick_Front":
                    # Composite action: move to pose with modified pitch and close gripper
                    if isinstance(value, dict):
                        pose_name = value.get("pose")
                    else:
                        pose_name = value

                    if pose_name in self.pose_map:
                        # Create a sequence for the two-step approach
                        pick_front_sequence = BehaviorModuleSequence()
                        
                        # First pose: move back
                        pose_cfg_1 = deepcopy(self.pose_map[pose_name])
                        pose_cfg_1.pose.x = pose_cfg_1.pose.x - 0.1
                        pose_cfg_1.pose.pitch = -1.57
                        pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_1))
                        
                        # Second pose: adjust pitch
                        pose_cfg_2 = deepcopy(self.pose_map[pose_cfg_1])
                        pose_cfg_2.pose.z = pose_cfg_2.pose.z - 0.08
                        pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_2))

                        # Second pose: adjust pitch
                        pose_cfg_3 = deepcopy(self.pose_map[pose_cfg_2])
                        pose_cfg_3.pose.x = pose_cfg_3.pose.x + 0.02
                        pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_3))
                        
                        # Add gripper action
                        pick_front_sequence.add_child(gripper("close"))

                        pose_cfg_4 = deepcopy(self.pose_map[pose_cfg_3])
                        pose_cfg_4.pose.x = pose_cfg_3.pose.x - 0.03
                        
                        pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_2))

                        sequence.add_child(pick_front_sequence)
                    else:
                        raise ValueError(f"Invalid pose name: {pose_name}")
                elif key == "Pick_Right":
                    # Composite action: move to pose from the Right and close gripper
                    if isinstance(value, dict):
                        pose_name = value.get("pose")
                    else:
                        pose_name = value

                    if pose_name in self.pose_map:
                        # Create a sequence for the Right approach
                        pick_Right_sequence = BehaviorModuleSequence()
                        
                        # First pose: move to the Right
                        pose_cfg_1 = deepcopy(self.pose_map[pose_name])
                        pose_cfg_1.pose.y = pose_cfg_1.pose.y - 0.1
                        pose_cfg_1.pose.yaw = 1.57  # Rotate 90 degrees to face the Right
                        pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_1))
                        
                        # Second pose: adjust position for Right approach
                        pose_cfg_2 = deepcopy(self.pose_map[pose_cfg_1])
                        pose_cfg_2.pose.x = pose_cfg_2.pose.x - 0.05
                        pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_2))

                        # Third pose: approach the object
                        pose_cfg_3 = deepcopy(self.pose_map[pose_cfg_2])
                        pose_cfg_3.pose.y = pose_cfg_3.pose.y + 0.08
                        pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_3))
                        
                        # Add gripper action
                        pick_Right_sequence.add_child(gripper("close"))

                        # Fourth pose: retreat with object
                        pose_cfg_4 = deepcopy(self.pose_map[pose_cfg_3])
                        pose_cfg_4.pose.y = pose_cfg_4.pose.y - 0.05
                        pose_cfg_4.pose.z = pose_cfg_4.pose.z + 0.03
                        pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_4))

                        sequence.add_child(pick_Right_sequence)
                    else:
                        raise ValueError(f"Invalid pose name: {pose_name}")
                elif key == "Place":
                    # Composite action: move to pose and open gripper
                    if isinstance(value, dict):
                        pose_name = value.get("pose")
                    else:
                        pose_name = value

                    sequence.add_child(self.pose_map.move2pose_module(pose_name))
                    sequence.add_child(gripper("open"))
                elif key == "PlaceObject":
                    # Use the unified primitive that combines movement and gripper control
                    if isinstance(value, dict):
                        pose_name = value.get("pose")
                    else:
                        pose_name = value
                        
                    sequence.add_child(self.pose_map.place_object_module(pose_name))
                else:
                    raise ValueError(f"Unknown target action: {key}")
            else:
                raise ValueError("Invalid format in target_pose. Each entry must be a dictionary.")
                
        return sequence
        
    def execute_experiment(self, experiment_name: str) -> BehaviorModuleSequence:
        """
        Load and create a behavior sequence for an experiment.
        
        Args:
            experiment_name (str): Name of the experiment to execute
            
        Returns:
            BehaviorModuleSequence: Sequence of behavior modules to execute
            
        Raises:
            FileNotFoundError: If the experiment file doesn't exist
            ValueError: If there's an error processing the experiment configuration
        """
        _, target_pose = self.load_experiment(experiment_name)
        return self.create_behavior_sequence(target_pose)
        
    def create_pick_and_place_sequence(self, pick_pose: str, place_pose: str, 
                                     clearance: float = 0.10, height: float = 0.10) -> BehaviorModuleSequence:
        """
        Create a sequence for picking an object from one pose and placing it at another.
        
        Args:
            pick_pose (str): Name of the pose where the object is located
            place_pose (str): Name of the pose where the object should be placed
            clearance (float, optional): Distance to move before/after picking/placing. Defaults to 0.10.
            height (float, optional): Height to lift/lower the object. Defaults to 0.10.
            
        Returns:
            BehaviorModuleSequence: Sequence of behavior modules for pick-and-place operation
            
        Raises:
            ValueError: If either pick_pose or place_pose is invalid
        """
        sequence = BehaviorModuleSequence()
        
        # Add pick sequence
        sequence.add_child(self.pose_map.pick_module(pick_pose, clearance, height))
        
        # Add place sequence
        sequence.add_child(self.pose_map.place_module(place_pose, clearance, height))
        
        return sequence
        
    def get_current_experiment(self) -> Optional[str]:
        """
        Get the name of the currently loaded experiment.
        
        Returns:
            Optional[str]: Name of the current experiment, or None if no experiment is loaded
        """
        return self.current_experiment 