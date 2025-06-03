"""
Default action configurations for the behavior tree framework.
This file registers the default actions with the action_mapping.
"""

from .action_mappings import action_mapping

def register_default_actions():
    """Register all default actions with the action mapping."""
    
    # Register move actions
    action_mapping.register_action("move2pose", action_mapping.create_move_to_pose)
    action_mapping.register_action("Move", action_mapping.create_move_to_pose)
    
    # Register pick actions
    action_mapping.register_action("Pick", action_mapping.create_pick_object)
    action_mapping.register_action("PickObject", action_mapping.create_pick_object)
    action_mapping.register_action("PickFront", action_mapping.create_pick_front)
    action_mapping.register_action("PickRight", action_mapping.create_pick_right)
    action_mapping.register_action("PickLeft", action_mapping.create_pick_left)
    action_mapping.register_action("PickRear", action_mapping.create_pick_rear)
    
    # Register place actions
    action_mapping.register_action("Place", action_mapping.create_place_object)
    action_mapping.register_action("PlaceObject", action_mapping.create_place_object)
    
    # Register utility actions
    action_mapping.register_action("rotate", action_mapping.create_rotate)
    action_mapping.register_action("gripper", action_mapping.create_gripper)
    action_mapping.register_action("single_joint_state", action_mapping.create_joint_state)

# Register default actions when this module is imported
register_default_actions() 