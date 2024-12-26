from ..file_utils.formats.yaml import YamlFormat
from .ConfigLoaderBase import ConfigLoaderBase
from dataclasses import dataclass
from pathlib import Path
import os

CONFIG_PATH = Path(os.environ["RAS_APP_PATH"])/"configs"
CONFIG_FILE = CONFIG_PATH/"ras_conf.yaml"

@dataclass
class RealConfig(ConfigLoaderBase):
    real: bool
    sim: bool

@dataclass
class FTPConfig(ConfigLoaderBase):
    ip: str
    port: int

@dataclass
class FTPsConfig(ConfigLoaderBase):
    sim : FTPConfig
    real : FTPConfig

@dataclass
class MQTTConfig(ConfigLoaderBase):
    ip: str
    port: int

@dataclass
class TransportConfig(ConfigLoaderBase):
    ftp: FTPsConfig
    mqtt: MQTTConfig

@dataclass
class RasConfig(ConfigLoaderBase):
    real: RealConfig
    transport: TransportConfig

class RasObject(object):
    _initialized = False
    ras : RasConfig = None
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        yaml_obj = YamlFormat.load(YamlFormat.load(CONFIG_FILE)["ras"])
        cls.ras = RasConfig.from_dict(yaml_obj)
        cls._initialized = True
