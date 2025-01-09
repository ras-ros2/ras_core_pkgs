#!/usr/bin/env python3
from ras_common.config.loaders.ConfigLoaderBase import ConfigLoaderBase
from ras_common.config.loaders.lab_setup import LabSetup
from dataclasses import dataclass
# Example usage
@dataclass
class MyConfig(ConfigLoaderBase):
    param1: float
    param2: str

@dataclass
class MyConfig2(ConfigLoaderBase):
    par1: int
    par2: MyConfig

config_data = {
    'par1': 42,
    'par2': {
        'param1': 42,
        'param2': 'hello'}
}
def main():
    LabSetup.init()
    print(LabSetup.lab_name)
    # config_instance = MyConfig2.from_dict(config_data)
    # print(config_instance)
    # print(MyConfig2.to_dict())

if __name__ == "__main__":
    main()