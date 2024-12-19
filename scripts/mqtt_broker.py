#!/usr/bin/env python3
from ras_common.transport.TransportServer import get_mqtt_broker_command
import subprocess
def main():
    subprocess.run(get_mqtt_broker_command(), shell=True)

if __name__ == '__main__':
    main()