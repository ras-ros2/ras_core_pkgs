import os
import yaml
from typing import Dict, List, Optional, Tuple
from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.keywords import TargetPoseMap, rotate, gripper, joint_state
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.lab_setup import LabSetup
from ras_common.config.loaders.objects import ObjectTypes
from .behavior_utility.yaml_parser import read_yaml_to_pose_dict

class ExperimentsService:
    """
    A service class for managing and executing robot experiments.
    
    This class handles loading experiment configurations from YAML files,
    managing poses and targets, and creating behavior sequences for robot actions.
    
    Attributes:
        pose_map (TargetPoseMap): Map of registered poses and their configurations
        experiments_dir (str): Directory containing experiment YAML files
    """
    
    def __init__(self, experiments_dir: str):
        """
        Initialize the ExperimentsService.
        
        Args:
            experiments_dir (str): Path to the directory containing experiment YAML files
        """
        self.pose_map = TargetPoseMap()
        self.experiments_dir = experiments_dir
        
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
                elif key == "gripper":
                    sequence.add_child(gripper(value))
                elif key == "rotate":
                    sequence.add_child(rotate(value))
                elif key == "joint_state":
                    sequence.add_child(joint_state(value))
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