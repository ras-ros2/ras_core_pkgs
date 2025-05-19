from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.primitives import MoveToPose,RotateEffector,Trigger, MoveToJointState, PlaceObject, PickObject, PickFront, PickRight, PickLeft, PickRear
from ..behaviors.gen_primitives import Pick as PickPrimitive
from ..behaviors.gen_primitives import Place as PlacePrimitive
from ..behaviors.modules import PickSequence,PlaceSequence
from typing import List,Dict
from ras_common.config.loaders.objects import ObjectTypes
from .ports import PortPoseCfg
from copy import deepcopy

class TargetPoseMap(object):
    """
    A class to manage and execute robot poses and sequences.
    
    This class provides functionality to register poses and create behavior modules
    for various robot actions including moving to poses, picking, and placing objects.
    
    Attributes:
        pose_map (dict): Dictionary mapping pose names to their corresponding pose configurations
    """
    
    def __init__(self):
        self.pose_map = {}
    
    def register_pose(self,pose_name,pose):
        """
        Register a new pose with the pose map.
        
        Args:
            pose_name (str): Name identifier for the pose
            pose: Pose configuration to be registered
        """
        self.pose_map[pose_name] = pose
    
    def move2pose_module(self,pose:str):
        """
        Create a MoveToPose module for moving to a specific pose.
        
        Args:
            pose (str): Name of the registered pose to move to
            
        Returns:
            MoveToPose: Behavior module for moving to the specified pose
            
        Raises:
            ValueError: If pose name is invalid or pose input type is incorrect
        """
        if (isinstance(pose,str)):
            if pose in self.pose_map:
                return MoveToPose(i_pose=self.pose_map[pose])
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")
        
    def move2pose_sequence_module(self,poses:List[str]):
        """
        Create a sequence of MoveToPose modules for multiple poses.
        
        Args:
            poses (List[str]): List of pose names to move to in sequence
            
        Returns:
            BehaviorModuleSequence: Sequence of move-to-pose behaviors
        """
        move2pose_sequence = BehaviorModuleSequence()
        move2pose_sequence.add_children([self.move2pose_module(pose) for pose in poses])
        return move2pose_sequence
    
    def pick_module(self, pose_or_name: str, target_name: str = None, grasp_frame: str = "grasp", pre_grasp_offset: float = 0.1):
        """
        Create a pick behavior module. Supports two styles:
        1. Direct PickPrimitive (old style): give only pose name.
        2. Sequence style: give name and target_name (used to move to pose and close gripper).
        
        Args:
            pose_or_name (str): If target_name is None, treated as pose name for PickPrimitive. 
                                Otherwise treated as sequence name.
            target_name (str, optional): Target pose name to move to in the sequence.
            grasp_frame (str): Grasp frame for PickPrimitive (used only if target_name is None)
            pre_grasp_offset (float): Pre-grasp offset (used only if target_name is None)
        
        Returns:
            BehaviorModuleSequence or PickPrimitive: Pick behavior module
        
        Raises:
            ValueError: If the pose is invalid
        """
        if target_name is None:
            # Handle PickPrimitive version
            if pose_or_name not in self.pose_map:
                raise ValueError(f"Invalid pose name: {pose_or_name}")
            pose_cfg = deepcopy(self.pose_map[pose_or_name])
            return PickPrimitive(i_target_pose=pose_cfg, i_grasp_frame=grasp_frame, i_pre_grasp_offset=pre_grasp_offset)
        else:
            # Handle sequence version
            seq = BehaviorModuleSequence(pose_or_name)
            seq.add_child(self.move2pose_module(target_name))
            seq.add_child(gripper(True))  # False = close gripper
            return seq

    def place_module(self, pose_or_name: str, target_name: str = None, grasp_frame: str = "grasp", pre_grasp_offset: float = 0.1):
        """
        Create a place behavior module. Supports two styles:
        1. Direct PlacePrimitive (old style): give only pose name.
        2. Sequence style: give name and target_name (used to move to pose and close gripper).
        
        Args:
            pose_or_name (str): If target_name is None, treated as pose name for PlacePrimitive. 
                                Otherwise treated as sequence name.
            target_name (str, optional): Target pose name to move to in the sequence.
            grasp_frame (str): Grasp frame for PlacePrimitive (used only if target_name is None)
            pre_grasp_offset (float): Pre-grasp offset (used only if target_name is None)
        
        Returns:
            BehaviorModuleSequence or PlacePrimitive: Place behavior module
        
        Raises:
            ValueError: If the pose is invalid
        """
        if target_name is None:
            # Handle PlacePrimitive version
            if pose_or_name not in self.pose_map:
                raise ValueError(f"Invalid pose name: {pose_or_name}")
            pose_cfg = deepcopy(self.pose_map[pose_or_name])
            return PlacePrimitive(i_target_pose=pose_cfg, i_grasp_frame=grasp_frame, i_pre_grasp_offset=pre_grasp_offset)
        else:
            # Handle sequence version
            seq = BehaviorModuleSequence(pose_or_name)
            seq.add_child(self.move2pose_module(target_name))
            seq.add_child(gripper(False))  # False = close gripper
            return seq

    def place_object_module(self, pose:str, grip_state:bool=False):
        """
        Create a PlaceObject module for combined move and gripper control.
        
        This is a single step primitive that both moves to the target pose and
        controls the gripper (typically opening it for placing objects).
        
        Args:
            pose (str): Name of the registered pose to move to
            grip_state (bool, optional): Gripper state (True=close, False=open). 
                                        Defaults to False for placing.
            
        Returns:
            PlaceObject: Combined behavior module for placing objects
            
        Raises:
            ValueError: If pose name is invalid
        """
        if isinstance(pose, str):
            if pose in self.pose_map:
                return PlaceObject(i_pose=self.pose_map[pose], i_grip_state=grip_state)
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")

    def pick_object_module(self, pose:str, grip_state:bool=True):
        """
        Create a PickObject module for combined move and gripper control.
        
        This is a single step primitive that both moves to the target pose and
        controls the gripper (typically closing it for picking objects).
        
        Args:
            pose (str): Name of the registered pose to move to
            grip_state (bool, optional): Gripper state (True=close, False=open). 
                                        Defaults to True for picking.
            
        Returns:
            PickObject: Combined behavior module for picking objects
            
        Raises:
            ValueError: If pose name is invalid
        """
        if isinstance(pose, str):
            if pose in self.pose_map:
                return PickObject(i_pose=self.pose_map[pose], i_grip_state=grip_state)
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")

    def pick_front_module(self, pose:str, grip_state:bool=True):
        """
        Create a PickFront module for combined move and gripper control.
        
        This is a single step primitive that both moves to the target pose and
        controls the gripper (typically closing it for picking objects).
        
        Args:
            pose (str): Name of the registered pose to move to
            grip_state (bool, optional): Gripper state (True=close, False=open). 
                                        Defaults to True for picking.
            
        Returns:
            PickFront: Combined behavior module for picking objects
            
        Raises:
            ValueError: If pose name is invalid
        """
        if isinstance(pose, str):
            if pose in self.pose_map:
                return PickFront(i_pose=self.pose_map[pose], i_grip_state=grip_state)
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")
    
    def pick_right_module(self, pose:str, grip_state:bool=True):
        """
        Create a PickRight module for combined move and gripper control.
        
        This is a single step primitive that both moves to the target pose and
        controls the gripper (typically closing it for picking objects).
        
        Args:
            pose (str): Name of the registered pose to move to
            grip_state (bool, optional): Gripper state (True=close, False=open). 
                                        Defaults to True for picking.
            
        Returns:
            PickRight: Combined behavior module for picking objects
            
        Raises:
            ValueError: If pose name is invalid
        """
        if isinstance(pose, str):
            if pose in self.pose_map:
                return PickRight(i_pose=self.pose_map[pose], i_grip_state=grip_state)
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")

    def pick_left_module(self, pose:str, grip_state:bool=True):
        """
        Create a PickLeft module for combined move and gripper control.
        This is a single step primitive that both moves to the target pose and
        controls the gripper (typically closing it for picking objects).
        Args:
            pose (str): Name of the registered pose to move to
            grip_state (bool, optional): Gripper state (True=close, False=open). 
                                        Defaults to True for picking.
        Returns:
            PickLeft: Combined behavior module for picking objects
        Raises:
            ValueError: If pose name is invalid
        """
        if isinstance(pose, str):
            if pose in self.pose_map:
                return PickLeft(i_pose=self.pose_map[pose], i_grip_state=grip_state)
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")

    def pick_rear_module(self, pose:str, grip_state:bool=True):
        """
        Create a PickRear module for combined move and gripper control.
        This is a single step primitive that both moves to the target pose and
        controls the gripper (typically closing it for picking objects).
        Args:
            pose (str): Name of the registered pose to move to
            grip_state (bool, optional): Gripper state (True=close, False=open). 
                                        Defaults to True for picking.
        Returns:
            PickRear: Combined behavior module for picking objects
        Raises:
            ValueError: If pose name is invalid
        """
        if isinstance(pose, str):
            if pose in self.pose_map:
                return PickRear(i_pose=self.pose_map[pose], i_grip_state=grip_state)
            else:
                raise ValueError(f"Invalid pose name {pose}")
        else:
            raise ValueError(f"Invalid pose input type {type(pose)}")

def rotate(angle:float):
    """
    Create a rotation behavior module.
    
    Args:
        angle (float): Angle to rotate by in radians
        
    Returns:
        RotateEffector: Behavior module for rotating the end effector
    """
    return RotateEffector(i_rotation_angle=angle)

def gripper(open:bool):
    """
    Create a gripper control behavior module.
    
    Args:
        open (bool): True to open gripper, False to close gripper
        
    Returns:
        Trigger: Behavior module for controlling the gripper
    """
    return Trigger(i_trigger=open)

def joint_state(joints:list):
    """
    Create a behavior module to move to a specific joint configuration.
    
    Args:
        joints (list): List of joint angles in radians
        
    Returns:
        MoveToJointState: Behavior module for moving to the specified joint configuration
        
    Raises:
        ValueError: If the number of joints doesn't match the robot's configuration
    """
    from ras_common.config.loaders.lab_setup import LabSetup
    LabSetup.init()
    joint_names = list(LabSetup.robot_config.home_joint_state.keys())
    if len(joints) != len(joint_names): 
        raise ValueError(f"Invalid number of joints {len(joints)}")
    joint_state = ",".join([f"{joint_names[i]}:{joints[i]}" for i in range(len(joints))])
    return MoveToJointState(i_joint_state=joint_state)

def single_joint_state(joint_index:int, joint_value:float):
    """
    Create a behavior module to move a single joint to a specific angle.
    
    Args:
        joint_index (int): Index of the joint to move (0-5 for a 6-joint robot)
        joint_value (float): Joint angle in radians
        
    Returns:
        MoveToJointState: Behavior module for moving the specified joint
        
    Raises:
        ValueError: If the joint index is out of range
        
    Example:
        # Move the 3rd joint (index 2) to 1.5 radians
        module = single_joint_state(2, 1.5)
        
        # In YAML experiment file:
        # targets:
        #   - single_joint_state: [2, 1.5]  # Move 3rd joint to 1.5 radians
    """
    from ras_common.config.loaders.lab_setup import LabSetup
    LabSetup.init()
    joint_names = list(LabSetup.robot_config.home_joint_state.keys())
    
    if joint_index < 0 or joint_index >= len(joint_names):
        raise ValueError(f"Joint index {joint_index} out of range. Valid range: 0-{len(joint_names)-1}")
    
    # Create a string with only the specified joint
    joint_state = f"{joint_names[joint_index]}:{joint_value}"
    
    return MoveToJointState(i_joint_state=joint_state)

# Mapping of keyword functions to their implementations
keyword_mapping = {
    "rotate": rotate,
    "gripper": gripper,
    'joint_state': joint_state,
    'single_joint_state': single_joint_state,
}