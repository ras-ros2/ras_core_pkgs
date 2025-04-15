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

from .module import BehaviorModule,BehaviorModuleSequence
from .instruction import PrimitiveInstruction,FunctionalInstruction
from typing import Dict,Iterable,List,Callable
from ras_common.config.loaders.ConfigLoaderBase import ConfigLoaderBase
from collections import OrderedDict
import inspect
import dataclasses

class KeywordInput(BehaviorModule):
    def __init__(self, kw_params):
        super().__init__()
        if len(self.output_port_names) != 0:
            raise ValueError("KeywordInputs has no output ports")
        self._check_ports(self.input_port_names,kw_params)
        self.input_ports = kw_params

def as_keyword_input(behavior_type):
    assert issubclass(behavior_type, BehaviorModule)
    assert not issubclass(behavior_type, KeywordInput)
    new_name = behavior_type.__name__
    new_class = type(new_name, (behavior_type, KeywordInput), dict())
    def new_init(self, kw_params):
        super(new_class, self).__init__()
        KeywordInput.__init__(self, kw_params)
    new_class.__init__ = new_init
    return new_class

@dataclasses.dataclass
class KeyWordParam(object):
    name: str
    type: type
    default: object
    param: inspect.Parameter

def keyword2module(keyword: Callable,identifier:str = None, params: object|Dict[str,object]|List[object] = None):
    if not callable(keyword):
        raise ValueError(f"Invalid keyword type: {identifier} {type(keyword)}")
    if not isinstance(identifier,str):
        identifier = keyword.__name__
    kw_sig = inspect.signature(keyword)
    params_decl : Dict[str,KeyWordParam] = OrderedDict()
    for param in kw_sig.parameters.values():
        if str(param).startswith('*'):
            raise ValueError(f"keyword {identifier} expects an invalid ambiguous parameter {param}")
        params_decl[param.name] = KeyWordParam(name=param.name,type=param.annotation if param.annotation != param.empty else None,
                                default=param.default if param.default != param.empty else None ,param=param)
    param_def = dict()
    behavior_module = None
    if isinstance(params, type(None)):
        return keyword2module(keyword,identifier,dict())
    elif (not isinstance(params,Iterable)) or (isinstance(params,str)):
        _key = next(iter(params_decl))
        return keyword2module(keyword,identifier,{_key:params})
    elif isinstance(params,Iterable):
        if len(params_decl.keys())==1:
            _key = next(iter(params_decl.keys()))
            if not isinstance(params,dict):
                if len(params) > 1:
                    raise ValueError(f"Keyword {identifier} expected 1 param, but got {len(params)}")
                return keyword2module(keyword,identifier,{_key:params})

        if not isinstance(params,dict):
            _max_len = len(params)
            if _max_len > len(params_decl):
                raise ValueError(f"Keyword {identifier} expected {len(params_decl)} params, but got {len(params)}")
            _iter = iter(params_decl.items())
            for _i in range(_max_len):
                _n,_p = next(_iter)
                param_def[_n] = params[_i]
            return keyword2module(keyword,identifier,param_def)
        else:
            undef_params = set()
            for _n,_p in params_decl.items():
                if _n in params.keys():
                    param_def[_n] = params[_n]
                elif isinstance(_p.default,type(None)):
                    undef_params.add(_n)
                else:
                    param_def[_n] = _p.default
            if len(undef_params)!=0:
                raise ValueError(f"undefined params for keyword {identifier}: {params}")
        behavior_module = keyword(**param_def)
    else:
        raise ValueError(f"Invalid keyword parameters: {params}")
    if not isinstance(behavior_module, BehaviorModule):
        raise ValueError(f"Invalid keyword type {identifier}, expected BehaviorModule but got: {type(behavior_module)}")
    return behavior_module