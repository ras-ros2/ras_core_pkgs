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

from  dataclasses import dataclass, field
from py_trees.behaviour import Behaviour
from py_trees.composites import Composite,Sequence
from .port import Port,RefPortEntry
from abc import ABC,abstractmethod
from typing import List,ClassVar,Set,Dict
import re
from geometry_msgs.msg import Pose

@dataclass(kw_only=True)
class BehaviorBase(Behaviour,ABC):
    type_name: ClassVar[str] = field(default=None)
    
    @classmethod
    def get_type_info(cls):
        if cls.type_name is None:
            return cls.__name__
        return cls.type_name
    
    def update(self):
        return
    
    def add_child(self, child):
        return


@dataclass(kw_only=True)
class BehaviorModule(BehaviorBase,ABC):
    @staticmethod
    def _check_ports(decl_set:set,def_dict:dict):
        if "name" in decl_set:
            raise ValueError(f"port_name 'name' is reserved for unique id")
        def_set = set(def_dict.keys())
        if not def_set.issuperset(decl_set):
            raise ValueError(f"Missing ports: {decl_set - def_set}")
        undecl_keys = def_set - decl_set
        if len(undecl_keys) > 0:
            print(f"WARNING: Undeclared ports: {undecl_keys}")
            for key in undecl_keys:
                del def_dict[key]
            

    def __post_init__(self):
        self.uid = self.get_type_info()
        self._input_port_names = set()
        self._output_port_names = set()
        self._input_ports = {}
        self._output_ports = {}

        for _k in self.__dict__.keys():
            if _k.startswith("i_"):
                self._input_port_names.add(_k[2:])
            elif _k.startswith("o_"):
                self._output_port_names.add(_k[2:])

        if len(self._input_port_names.intersection(self._output_port_names)) > 0:
            raise ValueError(f"Duplicate ports: {self._input_port_names.intersection(self._output_port_names)}")
        for _k, _v in self.__dict__.items():
            if _k.startswith("i_"):
                self._input_ports[_k[2:]] = _v
            elif _k.startswith("o_"):
                self._output_ports[_k[2:]] = _v
            
        self._check_ports(self._input_port_names, self._input_ports)
        self._check_ports(self._output_port_names, self._output_ports)


    def get_port_map(self):
        return {**self._input_ports,**self._output_ports}

@dataclass
class BehaviorModuleCollection(Composite,BehaviorModule):
    children: List[BehaviorModule] = field(default_factory=list)
    out_children: List[BehaviorModule] = field(default_factory=list,init=False)

    def __post_init__(self):
        super().__post_init__()
        from .instruction import ScriptInstruction
        for _port_name,_value in self._output_ports.items():
            if isinstance(_value,RefPortEntry):
                self.out_children.append(ScriptInstruction(code=_value.ref_serialize()))
            else:
                raise ValueError(f"Invalid output port type: {type(_value)} for {_port_name}.\n Output ports of collection should be of type {RefPortEntry}")

    def add_children(self, children: List[BehaviorModule]):
        if isinstance(children, list):
            for child in children:
                self.add_children(child)
        elif isinstance(children, BehaviorModule):
            self.children.append(children)
        else:
            raise ValueError(f"Invalid type of children: {type(children)}")

    def get_port_map(self):
        return BehaviorModule.get_port_map(self)

    def iterate(self):
        for child in self.children:
            yield child
        for child in self.out_children:
            yield child

    def add_child(self, child):
        return self.add_children(child)
    
    def add_children(self, children:List[BehaviorModule]|BehaviorModule):
        if isinstance(children,list):
            for child in children:
                self.add_children(child)
        elif isinstance(children,BehaviorModule):
            children.uid = children.get_type_info() + str(len(self.children))      
            self.children.append(children)
        else:
            raise ValueError(f"Invalid children type: {type(children)}")
            
        

@dataclass
class BehaviorModuleSequence(Sequence,BehaviorModuleCollection):
    pass