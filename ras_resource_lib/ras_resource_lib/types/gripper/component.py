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

from dataclasses import dataclass
from .._base._elements.component import OssComponent
from .._base.hardware.component import HardwareComponent
from .._base.sim_model.component import SimModelComponent
from .._base.mountable.component import MountableComponent
from .config import GripperCfg


@dataclass
class GripperComponent(MountableComponent,SimModelComponent,HardwareComponent):
    _internal_config: GripperCfg

    def __post_init__(self):
        self.trigger_functions = {func.__name__:func for func in self._internal_config.trigger_functions}

    def trigger(self,function_name:str,*args,**kwargs):
        if function_name in self.trigger_functions:
            func = self.trigger_functions[function_name]
            try:
                func(*args, **kwargs)
            except TypeError as e:
                raise ValueError(f"Incorrect arguments for function {function_name}: {e}")
        else:
            raise ValueError(f"Function {function_name} does not exist in trigger functions")
