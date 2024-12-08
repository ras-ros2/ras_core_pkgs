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
