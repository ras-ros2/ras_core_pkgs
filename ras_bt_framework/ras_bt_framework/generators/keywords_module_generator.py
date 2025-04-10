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

from ..behavior_template.keyword import KeywordInput,keyword2module
from typing import Dict,Iterable,List,Callable
from ..behavior_template.module import BehaviorModuleSequence,BehaviorModule
from ..behavior_template.instruction import FunctionalInstruction
from ..behaviors.keywords import keyword_mapping
import inspect


class KeywordModuleGenerator(object):
    def __init__(self,default_keywords:Dict[str,Callable]=keyword_mapping):
        self.registered_keywords : Dict[str,type[BehaviorModule]] = default_keywords

    def register(self,keyword: Callable|Dict[str,Callable] , name:str=None):
        if isinstance(keyword,dict):
            for _n,_k in keyword.items():
                self.register(_k,_n)
        elif callable(keyword):
            kw_sig = inspect.signature(keyword)
            for param in kw_sig.parameters.values():
                if str(param).startswith('*'):
                    raise ValueError(f"keyword {keyword} expects an invalid ambiguous parameter {param}") 
            if not isinstance(name,str):
                name = keyword.__name__
            self.registered_keywords[name] = keyword
        else:
            raise ValueError(f"Invalid keyword type: {keyword} {type(keyword)}")

    def generate(self,name,keyword_seq:List[Dict[str,dict]]):
        behavior_module : BehaviorModuleSequence= type(name,(BehaviorModuleSequence,),{})()
        for _kw in keyword_seq:
            identifier = None
            params = None
            if isinstance(_kw,str):
                identifier = _kw
                params = None
            elif isinstance(_kw,dict):
                if len(_kw)!=1:
                    raise ValueError(f"Invalid keyword input: {_kw}. Expected a single key-value pair")
                identifier,params = list(_kw.items())[0]
            else:
                raise ValueError(f"Invalid keyword input: {_kw}. Expected a single key-value pair")
            if identifier not in self.registered_keywords:
                raise ValueError(f"Unknown keyword: {identifier}")
            keyword = self.registered_keywords[identifier]
            kw_module = keyword2module(keyword,identifier,params)
            if not isinstance(kw_module,BehaviorModule):
                raise ValueError(f"Invalid keyword module: {kw_module}")
            behavior_module.add_children(kw_module)
        return behavior_module
