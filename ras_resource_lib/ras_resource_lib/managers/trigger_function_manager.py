
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
        

