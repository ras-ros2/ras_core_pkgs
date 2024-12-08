from dataclasses import dataclass
from enum import Enum

@dataclass 
class InteractableCfg(object):
    label: str
    link_name: str
    joint_name: str

@dataclass
class EquipmentCfg(object):
    label: str
    name: str
    interactables: list