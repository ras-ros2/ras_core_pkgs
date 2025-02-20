from ..file_utils.formats.yaml import YamlFormat
from .ConfigLoaderBase import ConfigLoaderBase
from dataclasses import dataclass
from pathlib import Path
import os
from ...globals import RAS_APP_PATH
CONFIG_PATH = Path(RAS_APP_PATH)/"configs"
CONFIG_FILE = CONFIG_PATH/"ras_conf.yaml"


@dataclass
class FileTransportCfg(ConfigLoaderBase):
    ip: str
    port: int

@dataclass
class FileTransportsCfg(ConfigLoaderBase):
    server : FileTransportCfg
    robot : FileTransportCfg

@dataclass
class MQTTConfig(ConfigLoaderBase):
    ip: str
    port: int

@dataclass
class TransportConfig(ConfigLoaderBase):
    implementation: str
    file_server: FileTransportsCfg
    mqtt: MQTTConfig

@dataclass
class RasConfig(ConfigLoaderBase):
    transport: TransportConfig

class RasObject(object):
    _initialized = False
    ras : RasConfig = None
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        yaml_obj = YamlFormat.load(CONFIG_FILE)["ras"]
        cls.ras = RasConfig.from_dict(yaml_obj)
        cls._initialized = True
