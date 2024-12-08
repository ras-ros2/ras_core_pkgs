from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
from typing import Dict
from enum import Enum


from .package_manager import get_asset_package_prefix,AssetType,get_asset_package_name

class SimResourceType(Enum):
    NONE = AssetType.NONE
    LAB = AssetType.LAB
    OBJECT = AssetType.OBJECT

class SimResourceManager(object):
    _initialized = False
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        cls._initialized = True
        cls._resource_providers : Dict[SimResourceType,Dict[str,set]] = dict()
        install_path = Path(os.environ['OSS_WORKSPACE_PATH'])/'install'
        assert install_path.is_dir()
        for type_name,resource_type in SimResourceType._member_map_.items():
            if type_name == "NONE":
                continue
            cls._resource_providers[resource_type]=dict()
            resource_provider_prefix = get_asset_package_prefix(resource_type)
            for package_path in install_path.glob(resource_provider_prefix+"*"):
                share_path = get_package_share_directory(package_path.name)
                if resource_type == SimResourceType.OBJECT:
                    model_path = Path(share_path)/'models'
                    provider_name = package_path.name.removeprefix(resource_provider_prefix)
                    cls._resource_providers[resource_type][provider_name] = set()
                    if(model_path.is_dir()):
                        for _path in model_path.iterdir():
                            if _path.is_dir() and (_path/'model.sdf').is_file() :
                                cls._resource_providers[resource_type][provider_name].add(_path.name)
                elif resource_type == SimResourceType.LAB:
                    world_path = Path(share_path)/'worlds'
                    provider_name = package_path.name.removeprefix(resource_provider_prefix)
                    cls._resource_providers[resource_type][provider_name] = set()
                    if(world_path.is_dir()):
                        for _file in world_path.iterdir():
                            if _file.is_file() and _file.name.endswith('.sdf'):
                                cls._resource_providers[resource_type][provider_name].add(_file.name.removesuffix(".sdf"))
        # print(cls._resource_providers)


    @classmethod
    def get_sim_resource(cls,res_type:SimResourceType,name:str):
        cls.init()
        if res_type == SimResourceType.NONE:
            raise ValueError(f"Invalid resource type {res_type}")
        res_table = cls._resource_providers[res_type]
        for provider,res_set in res_table.items():
            if name in res_set:
                provider_package_name = get_asset_package_name(res_type,provider)
                share_path = get_package_share_directory(provider_package_name)
                if res_type == SimResourceType.OBJECT:
                    model_path =  Path(share_path)/'models'/name/'model.sdf'
                    return str(model_path)
                elif res_type == SimResourceType.LAB:
                    world_path = Path(share_path)/'worlds'/(name+'.sdf')
                    return str(world_path)
        raise ValueError(f"Model {name} not found")