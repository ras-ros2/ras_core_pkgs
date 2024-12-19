import yaml
import os


class RasConfigLoader(object):
    _initialized = False
    ras = None
    @classmethod
    def init(cls):
        cls.load_config()
        if not cls._initialized:
            cls._initialized = True

    @classmethod
    def load_config(cls):
        conf = dict()
        with open(f"{os.environ['RAS_APP_PATH']}/configs/ras_conf.yaml", "r") as f:
            conf = yaml.safe_load(f)
        def add_attributes(obj:object, attributes:dict):
            for key, value in attributes.items():
                if isinstance(value, dict):
                    nested_obj = type(key+"_type", (object,), {_k:None for _k in value.keys()})()
                    if(not isinstance(getattr(obj, key),type(None))):
                        raise Exception(f"Attribute {key} not allowed for {obj}")
                    setattr(obj, key, nested_obj)
                    add_attributes(nested_obj, value)
                else:
                    setattr(obj, key, value)
            return obj
        add_attributes(cls,conf)
