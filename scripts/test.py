#!/usr/bin/env python3

from ras_common.config_loaders.RasConfigLoader import RasConfigLoader

def main():
    RasConfigLoader.init()
    print(dir(RasConfigLoader.ras))

if __name__ == "__main__":
    main()