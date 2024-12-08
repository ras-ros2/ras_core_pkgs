from ament_index_python import get_package_share_directory
from .sim_resource_manager import SimResourceManager,SimResourceType
# from .trigger_function_manager import TriggerFunctionManager
from .package_manager import PackageManager,AssetType

CONFIG_MODULE_NAME = "config"

class AssetManager(object):
    _initialized = False
    asset_types = {
        AssetType.GRIPPER,
        AssetType.MANIPULATOR,
        AssetType.LAB,
        AssetType.OBJECT
    }
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        cls.component_cache = dict()
        cls._initialized = True
        PackageManager.init()
        SimResourceManager.init()
        for asset_type in cls.asset_types:
            cls.component_cache[asset_type] = dict()

    @classmethod
    def get_asset_component(cls,label:str,asset_type:AssetType):
        cls.init()
        if asset_type in cls.asset_types:
            if label in cls.component_cache[asset_type]:
                return cls.component_cache[asset_type][label]
            if asset_type.name in SimResourceType._member_map_.values().mapping.keys():
                res_type = SimResourceType._member_map_.values().mapping[asset_type.name]
                return SimResourceManager.get_sim_resource(res_type,label)
            asset_module =  PackageManager.get_asset_module(asset_type,label,CONFIG_MODULE_NAME)
            loader_module = PackageManager.get_module(asset_type,"loader")
            component_loader = getattr(loader_module,f"{asset_type.name.capitalize()}Loader")
            asset_config_type = getattr(loader_module,f"{asset_type.name.capitalize()}Cfg")
            if hasattr(asset_module,"generate_configuration"):
                config = asset_module.generate_configuration()
                if isinstance(config,asset_config_type):
                    loader = component_loader(config)
                    component = loader.load()
                    cls.component_cache[asset_type][label]=component
                    return component
                else:
                    raise ValueError(f"Invalid configuration for {label} of type {asset_type}")
            raise ValueError(f"generate_configuration() method not found in {asset_module.__package__}")

        raise ValueError(f"Asset {label} of type {type} not found")