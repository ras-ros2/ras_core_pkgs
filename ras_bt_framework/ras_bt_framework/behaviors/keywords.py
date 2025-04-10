from ..behavior_template.module import BehaviorModuleSequence
from ..behaviors.primitives import MoveToPose,RotateEffector,Trigger, MoveToJointState
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

    def pick_module(self,pose:str,clearance:float=0.10,height:float=0.10):
        """
        Create a pick sequence module for grasping objects.
        
        Args:
            pose (str): Name of the pose where the object is located
            clearance (float, optional): Distance to move before/after picking. Defaults to 0.10.
            height (float, optional): Height to lift the object. Defaults to 0.10.
            
        Returns:
            PickSequence: Behavior module for picking sequence
            
        Raises:
            ValueError: If pose name is invalid
        """
        if pose in self.pose_map:
            return PickSequence(pose,clearance=clearance,height=height)
        else:
            raise ValueError(f"Invalid pose name {pose}")
    
    def place_module(self,pose:str,clearance:float=0.10,height:float=0.10):
        """
        Create a place sequence module for releasing objects.
        
        Args:
            pose (str): Name of the pose where the object should be placed
            clearance (float, optional): Distance to move before/after placing. Defaults to 0.10.
            height (float, optional): Height to lower the object. Defaults to 0.10.
            
        Returns:
            PlaceSequence: Behavior module for placing sequence
            
        Raises:
            ValueError: If pose name is invalid
        """
        if pose in self.pose_map:
            return PlaceSequence(pose,clearance=clearance,height=height)
        else:
            raise ValueError(f"Invalid pose name {pose}")

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
    joint_names = list(LabSetup.conf.robot.home_joint_state.keys())
    if len(joints) != len(joint_names): 
        raise ValueError(f"Invalid number of joints {len(joints)}")
    joint_state = ",".join([f"{joint_names[i]}:{joints[i]}" for i in range(len(joints))])
    return MoveToJointState(i_joint_state=joint_state)

# Mapping of keyword functions to their implementations
keyword_mapping = {
    "rotate": rotate,
    "gripper": gripper,
    'joint_state': joint_state
}