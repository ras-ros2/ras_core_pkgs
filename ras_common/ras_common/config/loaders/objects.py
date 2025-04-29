
import os
from pathlib import Path
from ..file_utils.formats.yaml import YamlFormat

from .ConfigLoaderBase import ConfigLoaderBase
from dataclasses import dataclass
import os 
from ...globals import RAS_CONFIGS_PATH
from typing import Dict
CONFIG_FILE = Path(RAS_CONFIGS_PATH)/"objects.yaml"

@dataclass
class ObjectConfig(ConfigLoaderBase):
    interaction_height: float
    max_height: float
    model: str

class ObjectTypes(object):
    _initialized = False
    conf : Dict[str,ObjectConfig] = dict()
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        yaml_obj = YamlFormat.load(CONFIG_FILE)["Objects"]
        for obj_name,obj_conf in yaml_obj.items():
            cls.conf[obj_name] = ObjectConfig.from_dict(obj_conf)
        cls._initialized = True
    @classmethod
    def get_object(cls,obj_name:str)->ObjectConfig:
        cls.init()
        if obj_name in cls.conf:
            return cls.conf[obj_name]
        else:
            raise ValueError(f"Invalid object name {obj_name}")


        
    