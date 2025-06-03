"""
Configuration file for action mappings in the behavior tree framework.
This file defines the mapping between action keywords and their corresponding primitive actions.
"""

from typing import Dict, Callable, Any
from ..behaviors.primitives import (
    MoveToPose, RotateEffector, Trigger, MoveToJointState,
    PlaceObject, PickObject, PickFront, PickRight, PickLeft, PickRear
)
from ..behavior_template.module import BehaviorModuleSequence

class ActionMapping:
    """
    A class to manage the mapping between action keywords and their corresponding primitive actions.
    """
    def __init__(self):
        self._mappings: Dict[str, Callable] = {}
        self._pose_map = {}

    def register_pose(self, pose_name: str, pose: Any) -> None:
        """Register a pose with the pose map."""
        self._pose_map[pose_name] = pose

    def register_action(self, keyword: str, action_func: Callable) -> None:
        """Register an action mapping."""
        self._mappings[keyword] = action_func

    def get_action(self, keyword: str) -> Callable:
        """Get the action function for a keyword."""
        if keyword not in self._mappings:
            raise ValueError(f"Unknown action keyword: {keyword}")
        return self._mappings[keyword]

    def get_pose(self, pose_name: str) -> Any:
        """Get a registered pose."""
        if pose_name not in self._pose_map:
            raise ValueError(f"Unknown pose: {pose_name}")
        return self._pose_map[pose_name]

    def create_move_to_pose(self, pose_name: str) -> MoveToPose:
        """Create a MoveToPose action for a registered pose."""
        return MoveToPose(i_pose=self.get_pose(pose_name))

    def create_rotate(self, angle: float) -> RotateEffector:
        """Create a RotateEffector action."""
        return RotateEffector(i_rotation_angle=angle)

    def create_gripper(self, state: bool) -> Trigger:
        """Create a Trigger action for gripper control."""
        return Trigger(i_trigger=state)

    def create_joint_state(self, joint_state: str) -> MoveToJointState:
        """Create a MoveToJointState action."""
        return MoveToJointState(i_joint_state=joint_state)

    def create_pick_object(self, pose_name: str, grip_state: bool = True) -> PickObject:
        """Create a PickObject action."""
        return PickObject(i_pose=self.get_pose(pose_name), i_grip_state=grip_state)

    def create_place_object(self, pose_name: str, grip_state: bool = False) -> PlaceObject:
        """Create a PlaceObject action."""
        return PlaceObject(i_pose=self.get_pose(pose_name), i_grip_state=grip_state)

    def create_pick_front(self, pose_name: str, grip_state: bool = True) -> PickFront:
        """Create a PickFront action."""
        return PickFront(i_pose=self.get_pose(pose_name), i_grip_state=grip_state)

    def create_pick_right(self, pose_name: str, grip_state: bool = True) -> PickRight:
        """Create a PickRight action."""
        return PickRight(i_pose=self.get_pose(pose_name), i_grip_state=grip_state)

    def create_pick_left(self, pose_name: str, grip_state: bool = True) -> PickLeft:
        """Create a PickLeft action."""
        return PickLeft(i_pose=self.get_pose(pose_name), i_grip_state=grip_state)

    def create_pick_rear(self, pose_name: str, grip_state: bool = True) -> PickRear:
        """Create a PickRear action."""
        return PickRear(i_pose=self.get_pose(pose_name), i_grip_state=grip_state)

# Create a global instance of ActionMapping
action_mapping = ActionMapping() 