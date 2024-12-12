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

from typing import Callable

class TriggerFunctionManager(object):
    _functions = dict()
    _initialized = False
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        cls._initialized = True 

    @classmethod
    def register(cls,gripper:str,func:Callable,label:str=None):
        if not isinstance(label,str):
            label=func.__name__
        if gripper in cls._functions:
            if label in cls._functions[gripper]:
                raise ValueError(f"Trigger function {label} already exists for gripper {gripper}")
        else:
            cls._functions[gripper] = dict()
        cls._functions[gripper][label] = func

    @classmethod
    def call_trigger(cls,gripper:str,label:str,*args,**kwargs):
        if gripper in cls._functions and label in cls._functions[gripper]:
            cls._functions[gripper][label](args,kwargs)
        else:
            raise ValueError(f"Trigger function {label} does not exist for gripper {gripper}")
        

