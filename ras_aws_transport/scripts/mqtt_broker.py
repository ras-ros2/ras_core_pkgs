#!/usr/bin/env python3
from ras_common.transport.TransportWrapper import run_mqtt_broker
import subprocess
def main():
    try:
        run_mqtt_broker()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()