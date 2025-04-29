

from abc import ABC,abstractmethod
import inspect,re,textwrap,os
from pathlib import Path
from dataclasses import dataclass, field
from typing import Callable

class PythonTemplate(ABC):
    def __init__(self,func):
        self.parameters = inspect.signature(func).parameters
        source_code = inspect.getsource(func)
        source_code = source_code.split(os.linesep,1)[1]
        self.source_code =  textwrap.dedent(source_code)
        self.definitions = dict()
    
    def set_keywords(self,*args,**kwargs):
        param_items = list(self.parameters.items())
        param_index = 0
        for _a in args:
            param_name,param = param_items[param_index]
            if (param.annotation!=inspect._empty):
                if not isinstance(_a,param.annotation):
                    raise TypeError(f"Argument {param_name} must be of type {param.annotation}")
            kwargs[param_name] = _a
            param_index +=1
        for param_name,param in param_items:
            if param_name not in kwargs:
                if (param.default==inspect.Parameter.empty):
                    raise TypeError(f"Missing argument {param_name}")
                else:
                    self.definitions[param_name]=param.default
            else:
                if param.annotation!=inspect._empty:
                    if not isinstance(kwargs[param_name],param.annotation):
                        raise TypeError(f"Argument {param_name} must be of type {param.annotation}")
                self.definitions[param_name]=kwargs[param_name]

    def create_file(self,file_path):
        preamble = os.linesep.join([f"{_param} = {repr(_value)}" for _param,_value in self.definitions.items()])
        out_str = f"{preamble}{os.linesep}{self.source_code}"
        file_path = Path(file_path)
        with file_path.open("w") as f:
            f.write(out_str)
            f.close()

@dataclass
class TemplateMap:
    config:Callable
    custom_map:dict
    file_tree:dict = field(default_factory=dict)