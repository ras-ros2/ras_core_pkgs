#!/usr/bin/env python3

"""
Copyright (C) 2025 Sachin Kumar
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.
You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
For inquiries or further information, you may contact:
Sachin Kumar
Email: info@opensciencestack.org
"""

import rclpy
from rclpy.node import Node
from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher
from ras_interfaces.srv import ReadBlack
import time
from enum import Enum

class TransportCommands(Enum):
    HOME = "home"
    SYNC = "sync"

class TransportServerService(Node):
    def __init__(self):
        super().__init__('transport_server_service')
        self.service_server = self.create_service(ReadBlack, '/server_transport', self.service_server_cb)

        self.mqtt_pub = TransportMQTTPublisher("server_transport")
        self.connect_aws()

    def connect_aws(self):
        self.mqtt_pub.connect_with_retries()

    def service_server_cb(self, request: ReadBlack.Request, response: ReadBlack.Response):
        if request.blackboard == TransportCommands.HOME.value:
            self.publish_with_retry(TransportCommands.HOME.value)
            response.success = True
        elif request.blackboard == TransportCommands.SYNC.value:
            self.publish_with_retry(TransportCommands.SYNC.value)
            response.success = True
        else:
            response.success = False
        return response

    def publish_with_retry(self, payload):
        self.get_logger().info("Publishing to MQTT")
        self.mqtt_pub.publish(payload)

        time.sleep(0.25)

def main(args=None):
    rclpy.init(args=args)
    node = TransportServerService()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.mqtt_pub.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.mqtt_pub.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
