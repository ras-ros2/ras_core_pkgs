from ..behavior_template.port import Port,PortData,PortEntry,RefPortEntry
from ..behavior_template.module import BehaviorModule,BehaviorModuleCollection
from enum import Enum

default_port_types = (bool,int,float,str)

class ModulePortType(Enum):
    INVALID = None
    INPUT = True
    OUTPUT = False

class PortMan(object):
    session_entries = set()
    def __init__(self):
        self.session_entries = set()

    def reset_session(self):
        self.session_entries.clear()

    def get_serialized_port_data(self,port_object:Port|object):
        port_type = type(port_object)
        if issubclass(port_type,(default_port_types)):
            return str(port_object)
        elif issubclass(port_type,(PortData,PortEntry,RefPortEntry)):
            return port_object.serialize()
        else:
            return None
        
    def get_serialized_module(self,behavior:BehaviorModule):
        common_ports = behavior._input_port_names.intersection(behavior._output_port_names)
        if len(common_ports)>0:
            raise ValueError(f"Duplicate ports not allowed : {common_ports}")
        if isinstance(behavior,BehaviorModule):
            def _process_ports_dict(_ports:dict,port_type:ModulePortType):
                for _k,_v in _ports.items():
                    if isinstance(_v,PortEntry):
                        if(port_type == ModulePortType.INPUT):
                            self.session_entries.add(_v.name)
                        elif(port_type == ModulePortType.OUTPUT):
                            if _v.name not in self.session_entries:
                                raise ValueError(f"PortEntry {_v.name} has not been defined yet for module {behavior.get_type_info()}")
                    if isinstance(_v,RefPortEntry):
                        if isinstance(behavior,BehaviorModuleCollection):
                            def _check_for_output_entry(_name,_behavior:BehaviorModuleCollection):
                                for _child in _behavior.iterate():
                                    if _name in _child._output_port_names:
                                        return True
                                    if isinstance(_child,BehaviorModuleCollection):
                                        if _check_for_output_entry(_name,_child):
                                            return True
                                return False
                            if not _check_for_output_entry(_v.reference.name,behavior):
                                raise ValueError(f"Reference {_v} is not an output port of any of children for {behavior}")
                        else:
                            raise ValueError(f"{RefPortEntry} is only allowed for modules of type {BehaviorModuleCollection}.")
                    data = self.get_serialized_port_data(_v)
                    if isinstance(data,type(None)):
                        raise ValueError(f"Unexpected type {type(_v)} for port {_k} of module {behavior.get_type_info()}")
                    _ports[_k] = data
            _process_ports_dict(behavior._input_ports,ModulePortType.INPUT)
            _process_ports_dict(behavior._output_ports,ModulePortType.OUTPUT)
        return behavior
