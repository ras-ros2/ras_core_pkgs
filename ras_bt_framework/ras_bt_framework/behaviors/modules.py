
import os
from .primitives import RotateEffector, SaySomething,ThinkSomethingToSay,MoveToPose,Trigger

from ..behavior_template.module import BehaviorModuleSequence
from geometry_msgs.msg import Pose

from ..behavior_utility.yaml_parser import read_yaml_to_pose_dict

from ament_index_python import get_package_share_directory
from ..behavior_template.port import PortData,PortEntry,RefPortEntry
from ..behaviors.ports import PortPoseCfg
from copy import deepcopy

class SaySomethingSequence(BehaviorModuleSequence):
    def __init__(self,message_entry):
        self.o_message = RefPortEntry(message_entry,"out_message")
        super().__init__()
        self.add_children([
            ThinkSomethingToSay(i_reference="Hello World!",o_message=PortEntry("out_message")),
            ])
        
class MyCustomSequence(BehaviorModuleSequence):
    def __init__(self):
        super().__init__()
        self.add_children([
            SaySomethingSequence(message_entry="my_message"),
            SaySomething(i_message=PortEntry("my_message")),
            ])

class PickObject(BehaviorModuleSequence):
    def __init__(self, sequence_list):
        super().__init__()
        self.add_children(sequence_list)

class RotateEffectorSequence(BehaviorModuleSequence):
    def __init__(self):
        super().__init__()
        self.add_children([
            RotateEffector(i_rotation_angle=1.8),
            ])

class PickSequence(BehaviorModuleSequence):
    def __init__(self,src_pose:PortPoseCfg,clearance:float,height:float):
        super().__init__()
        clearance_pose = deepcopy(src_pose)
        clearance_pose.pose.z += clearance
        pick_pose = deepcopy(src_pose)
        pick_pose.pose.z += height
        self.add_children([
            MoveToPose(i_pose=clearance_pose),
            MoveToPose(i_pose=src_pose),
            Trigger(i_trigger=True),
            MoveToPose(i_pose=pick_pose),
            ])

class PlaceSequence(BehaviorModuleSequence):
    def __init__(self,dest_pose:PortPoseCfg,clearance:float):
        super().__init__()
        clearance_pose = deepcopy(dest_pose)
        clearance_pose.pose.z += clearance
        self.add_children([
            MoveToPose(i_pose=clearance_pose),
            MoveToPose(i_pose=dest_pose),
            Trigger(i_trigger=False),
            MoveToPose(i_pose=clearance_pose),
            ])

class PressButton(BehaviorModuleSequence):
    def __init__(self,pose:Pose,travel_length):
        super().__init__()
        import numpy as np
        quaternion = np.array(quaternion)
        quaternion /= np.linalg.norm(quaternion)
        from scipy.spatial.transform import Rotation as R
        
        rot = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
        
        rotated_vector = rot.apply(np.array([1, 0, 0]))
        
        rotated_vector / np.linalg.norm(rotated_vector)
        
        pressed_pose = Pose()
        pressed_pose.position.x = pose.position.x + rotated_vector[0] * travel_length
        pressed_pose.position.y = pose.position.y + rotated_vector[1] * travel_length
        pressed_pose.position.z = pose.position.z + rotated_vector[2] * travel_length
        pressed_pose.orientation = pose.orientation

        self.add_children([
            MoveToPose(i_pose=",".join(map(str, pose))),
            MoveToPose(i_pose=",".join(map(str, pressed_pose))),
            ])
