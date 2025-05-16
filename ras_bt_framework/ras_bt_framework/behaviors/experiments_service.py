import os
import yaml
from typing import Dict, List, Optional, Tuple
from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.keywords import TargetPoseMap, rotate, gripper, joint_state
from ..behaviors.ports import PortPoseCfg
from ras_common.config.loaders.lab_setup import LabSetup
from ras_common.config.loaders.objects import ObjectTypes
from .behavior_utility.yaml_parser import read_yaml_to_pose_dict, read_yaml_to_target_dict
from ..behaviors.gen_primitives import Pick as PickPrimitive
from ..behaviors.gen_primitives import Pick_Front as PickFrontPrimitive
from ..behaviors.gen_primitives import Place as PlacePrimitive
from copy import deepcopy
from ..behaviors.gen_primitives import MoveToPose

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
                elif key == "Move":
                    sequence.add_child(self.pose_map.move2pose_module(value))
                elif key == "Pick":
                    if value not in self.pose_map:
                        raise ValueError(f"Invalid pose name for Pick: {value}")

                    pose_cfg = self.pose_map[value]
                    pick_module = PickPrimitive(
                        i_target_pose=pose_cfg,
                        i_grasp_frame="grasp",         # Default values as in parser
                        i_pre_grasp_offset=0.1
                    )
                    sequence.add_child(pick_module)
                elif key == "Pick_Front":
                    
                    if value not in self.pose_map:
                        raise ValueError(f"Invalid pose name for Pick_Front: {value}")

                    # First pose: move back
                    pose_cfg_1 = deepcopy(self.pose_map[value])
                    pose_cfg_1.pose.x = pose_cfg_1.pose.x - 0.1
                    pose_cfg_1.pose.pitch = -1.57
                    
                    # Second pose: adjust pitch
                    pose_cfg_2 = deepcopy(self.pose_map[pose_cfg_1])
                    pose_cfg_2.pose.z = pose_cfg_2.pose.z - 0.08

                    # Third pose: move back
                    pose_cfg_3 = deepcopy(self.pose_map[pose_cfg_2])
                    pose_cfg_3.pose.x = pose_cfg_3.pose.x + 0.02
                    
                    pose_cfg_4 = deepcopy(self.pose_map[pose_cfg_3])
                    pose_cfg_4.pose.x = pose_cfg_4.pose.x - 0.03
                        
                    # Create sequence with both poses
                    pick_front_sequence = BehaviorModuleSequence()
                    
                    # Add first move
                    pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_1))
                    
                    # Add second move
                    pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_2))

                    # Add third move
                    pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_3))
                    
                    # Add gripper action
                    pick_front_sequence.add_child(gripper(True))

                    pick_front_sequence.add_child(MoveToPose(i_pose=pose_cfg_4))
                    
                    sequence.add_child(pick_front_sequence)
                elif key == "Pick_Right":
                    
                    if value not in self.pose_map:
                        raise ValueError(f"Invalid pose name for Pick_Right: {value}")

                    # First pose: move to the Right
                    pose_cfg_1 = deepcopy(self.pose_map[value])
                    pose_cfg_1.pose.y = pose_cfg_1.pose.y - 0.1
                    pose_cfg_1.pose.yaw = 1.57  # Rotate 90 degrees to face the Right
                    
                    # Second pose: adjust position for Right approach
                    pose_cfg_2 = deepcopy(self.pose_map[pose_cfg_1])
                    pose_cfg_2.pose.x = pose_cfg_2.pose.x - 0.05

                    # Third pose: approach the object
                    pose_cfg_3 = deepcopy(self.pose_map[pose_cfg_2])
                    pose_cfg_3.pose.y = pose_cfg_3.pose.y + 0.08
                    
                    # Fourth pose: retreat with object
                    pose_cfg_4 = deepcopy(self.pose_map[pose_cfg_3])
                    pose_cfg_4.pose.y = pose_cfg_4.pose.y - 0.05
                    pose_cfg_4.pose.z = pose_cfg_4.pose.z + 0.03
                        
                    # Create sequence for Right approach
                    pick_Right_sequence = BehaviorModuleSequence()
                    
                    # Add first move
                    pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_1))
                    
                    # Add second move
                    pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_2))

                    # Add third move
                    pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_3))
                    
                    # Add gripper action
                    pick_Right_sequence.add_child(gripper(True))

                    # Add fourth move (retreat)
                    pick_Right_sequence.add_child(MoveToPose(i_pose=pose_cfg_4))
                    
                    sequence.add_child(pick_Right_sequence)
                elif key == "Place":
                    if value not in self.pose_map:
                        raise ValueError(f"Invalid pose name for Place: {value}")

                    pose_cfg = self.pose_map[value]
                    place_module = PlacePrimitive(
                        i_target_pose=pose_cfg,
                        i_grasp_frame="grasp",         # Default values as in parser
                        i_pre_grasp_offset=0.1
                    )
                    sequence.add_child(place_module)
                elif key == "PlaceObject":
                    if value not in self.pose_map:
                        raise ValueError(f"Invalid pose name for PlaceObject: {value}")
                    
                    # Use our new combined primitive
                    sequence.add_child(self.pose_map.place_object_module(value))
                elif key == "gripper":
                    sequence.add_child(gripper(value))
                elif key == "rotate":
                    sequence.add_child(rotate(value))
                elif key == "joint_state":
                    sequence.add_child(joint_state(value))
                elif key == "single_joint_state":
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