#!/usr/bin/env python3
from oss_resource_lib.managers.asset_manager import AssetManager,AssetType

def main():
    AssetManager.init()
    component =AssetManager.get_asset_component("oss_lab1",AssetType.LAB)
    print(component)

if __name__=="__main__":
    main()