from enum import Enum
from importlib import import_module

class AssetType(Enum):
    NONE = 0

    MANIPULATOR = 1
    GRIPPER = 2
    OBJECT = 3
    LAB = 4
    OPERATION = 5
    INTERACTION = 6
    EQUIPMENT = 7

def find_module(package_name: str,module_name: str):
    if module_name is None: 
        package_name,module_name = package_name.rsplit(".")
    try:
        module = import_module(module_name,package=package_name)
        print(module_name,package_name)
    except ImportError:
        print(package_name,module_name)
        module = import_module(package_name+f".{module_name}")
    return module

def get_asset_package_prefix(asset_type:AssetType):
    return f"ras_asset_{asset_type.name.lower()}_"

def get_asset_package_name(asset_type:AssetType,asset_name:str):
    return f"{get_asset_package_prefix(asset_type)}{asset_name}"

class PackageManager(object):
    _initialized = False
    @classmethod
    def init(cls):
        if(cls._initialized): return 
        cls._initialized = True

    @classmethod
    def get_module(cls,type:AssetType,module_name=None):
        if type != AssetType.NONE:
            if module_name is None:
                return find_module("ras_resource_lib.types",type.name.lower())
            else :
                return find_module(f"ras_resource_lib.types.{type.name.lower()}",module_name)
        else:
            raise ValueError("No module found")
        
    
    @classmethod
    def get_asset_module(cls,asset_type:AssetType,asset_name:str,module_name:str):
        return find_module(get_asset_package_name(asset_type,asset_name),module_name)
    

        
