"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

import xml.etree.ElementTree as ET
from ras_bt_framework.managers.primitive_action_manager import PrimitiveActionManager
from rclpy.node import Node
from ..behavior_template.module import BehaviorModule, BehaviorModuleSequence, BehaviorModuleCollection

from ..behaviors.primitives import MoveToPose, Trigger, RotateEffector, ExecuteTrajectory, LoggerClientTrigger, MoveToJointState, PlaceObject, PickObject, PickFront, PickRight, PickLeft, PickRear
from ..generators.behavior_tree_generator import BehaviorTreeGenerator
from copy import deepcopy
from dataclasses import dataclass

# Mapping of primitive types to their XML tag names
mapping = {
    "MoveToPose": "ExecuteTrajectory",
    "Trigger": "Trigger",
    "RotateEffector": "ExecuteTrajectory",
    "MoveToJointState": "MoveToJointState",
    "PickObject": "ExecuteTrajectory",  # Add mapping for PickObject
    "PickFront": "ExecuteTrajectory",
    "PickRight": "ExecuteTrajectory",
    "PickLeft": "ExecuteTrajectory",
    "PickRear": "ExecuteTrajectory"
}

@dataclass
class SequenceId:
    sequence: int = 1

    def inc(self):
        self.sequence += 1
    
    def get(self):
        return self.sequence
    
def update_bt(behavior: BehaviorModule, sequence=None):
    if sequence is None:
        sequence = SequenceId()
    if isinstance(behavior, BehaviorModuleSequence) or isinstance(behavior, BehaviorModuleCollection):
        new_children = list()
        for child in behavior.iterate():
            if isinstance(child, MoveToPose):
                new_child = ExecuteTrajectory(i_sequence =sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
            elif isinstance(child, Trigger):
                new_child = LoggerClientTrigger()
                new_children.append(new_child)
            elif isinstance(child, RotateEffector):
                new_child = ExecuteTrajectory(i_sequence =sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
            elif isinstance(child, PlaceObject):
                # Handle PlaceObject primitive - convert to appropriate execution sequence
                # First add the movement to the pose
                new_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
                
                # Then add the gripper control action
                # Extract the grip_state value from the PlaceObject primitive
                grip_state = child.i_grip_state
                new_children.append(Trigger(i_trigger=grip_state))
                new_children.append(LoggerClientTrigger())
            elif isinstance(child, PickObject):
                # Handle PickObject primitive - convert to appropriate execution sequence
                # First add the movement to the pose
                new_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
                
                # Then add the gripper control action
                # Extract the grip_state value from the PickObject primitive
                grip_state = child.i_grip_state
                new_children.append(Trigger(i_trigger=grip_state))
                new_children.append(LoggerClientTrigger())
            elif isinstance(child, PickFront):
                # Handle PickFront primitive - convert to appropriate execution sequence
                # First add the movement to the pose
                new_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

                # Second: move to the lowered pose (z - 0.1)
                lowered_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(lowered_pose_child)
                new_children.append(LoggerClientTrigger())
                
                # Then add the gripper control action
                # Extract the grip_state value from the PickFront primitive
                grip_state = child.i_grip_state
                new_children.append(Trigger(i_trigger=grip_state))
                new_children.append(LoggerClientTrigger())

                safe_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(safe_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

            elif isinstance(child, PickRight):
                # Handle PickFront primitive - convert to appropriate execution sequence
                # First add the movement to the pose
                new_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

                # Second: move to the lowered pose (z - 0.1)
                lowered_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(lowered_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
                
                # Then add the gripper control action
                # Extract the grip_state value from the PickFront primitive
                grip_state = child.i_grip_state
                new_children.append(Trigger(i_trigger=grip_state))
                new_children.append(LoggerClientTrigger())

                safe_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(safe_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

            elif isinstance(child, PickLeft):
                # Handle PickLeft primitive - convert to appropriate execution sequence
                # First add the movement to the pose
                new_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

                # Second: move to the lowered pose (z - 0.1)
                lowered_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(lowered_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
                
                # Then add the gripper control action
                # Extract the grip_state value from the PickLeft primitive
                grip_state = child.i_grip_state
                new_children.append(Trigger(i_trigger=grip_state))
                new_children.append(LoggerClientTrigger())

                safe_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(safe_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

            elif isinstance(child, PickRear):
                # Handle PickRear primitive - convert to appropriate execution sequence
                # First add the movement to the pose
                new_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(new_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

                # Second: move to the lowered pose (z - 0.1)
                lowered_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(lowered_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()
                
                # Then add the gripper control action
                # Extract the grip_state value from the PickRear primitive
                grip_state = child.i_grip_state
                new_children.append(Trigger(i_trigger=grip_state))
                new_children.append(LoggerClientTrigger())

                safe_pose_child = ExecuteTrajectory(i_sequence=sequence.get())
                new_children.append(safe_pose_child)
                new_children.append(LoggerClientTrigger())
                sequence.inc()

            else:
                raise ValueError(f"Invalid child type: {type(child)}")
        new_behavior = deepcopy(behavior)
        new_behavior.children = list()
        new_behavior.add_children(new_children)
    return new_behavior

            

# Function to update XML based on mapping
def update_xml(element, sequence=1):
    for child in element:
        if child.tag == "MoveToPose":
            # Replace the tag name
            child.tag = mapping.get("MoveToPose", child.tag)
            
            # Replace `pose` with `sequence` attribute
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1
        elif child.tag == "Trigger":
            # Keep the trigger tag but ensure it has all necessary attributes
            child.tag = mapping.get("Trigger", child.tag)
            # No attribute changes needed for Trigger
        elif child.tag == "PlaceObject":
            # Handle PlaceObject by splitting it into a move action and gripper action
            # First create the move action
            child.tag = "ExecuteTrajectory"
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1
            
            # Then create a Trigger action for gripper control (for XML editing)
            # This would need more complex XML manipulation which we won't do here
            # since we handle this in the update_bt function instead
        elif child.tag == "PickObject":
            # Handle PickObject by splitting it into a move action and gripper action
            # First create the move action
            child.tag = "ExecuteTrajectory"
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1
            
            # Then create a Trigger action for gripper control (for XML editing)
            # This would need more complex XML manipulation which we won't do here
            # since we handle this in the update_bt function instead
        
        elif child.tag == "PickFront":
            # Handle PickObject by splitting it into a move action and gripper action
            # First create the move action
            child.tag = "ExecuteTrajectory"
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1

        elif child.tag == "PickRight":
            # Handle PickObject by splitting it into a move action and gripper action
            # First create the move action
            child.tag = "ExecuteTrajectory"
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1

        elif child.tag == "PickLeft":
            # Handle PickLeft by splitting it into a move action and gripper action
            # First create the move action
            child.tag = "ExecuteTrajectory"
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1

        elif child.tag == "PickRear":
            # Handle PickRear by splitting it into a move action and gripper action
            # First create the move action
            child.tag = "ExecuteTrajectory"
            if "pose" in child.attrib:
                child.set("sequence", str(sequence))
                del child.attrib["pose"]
                sequence += 1

        # Recursively process child elements
        sequence = update_xml(child, sequence)
    return sequence

# Update the XML tree
# update_xml(root)

# Write the modified XML back to a file
# tree.write("updated_behavior_tree.xml", encoding="utf-8", xml_declaration=True)
