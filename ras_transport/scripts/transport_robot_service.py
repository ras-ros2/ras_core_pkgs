
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
from ras_common.transport.TransportWrapper import TransportMQTTSubscriber
from enum import Enum

class TransportCommands(Enum):
    HOME = "home"
    SYNC = "sync"

class TransportRobotService(Node):
    def __init__(self):
        super().__init__('transport_robot_service')
        self.mqtt_sub = TransportMQTTSubscriber("server_transport", self.custom_callback)
        self.connect_aws()

    def connect_aws(self):
        self.mqtt_sub.connect_with_retries()

    def custom_callback(self, message):
        payload =  message.decode("utf-8")
        self.get_logger().info(f"Received message: {payload}")
        self.execute_request(payload)

    def execute_request(self, payload):
        if payload == TransportCommands.HOME.value:
            self.get_logger().info("Executing Home command")
        elif payload == TransportCommands.SYNC.value:
            self.get_logger().info("Executing Sync command")
        else:
            self.get_logger().info("Invalid command")

def main(args=None):
    rclpy.init(args=args)
    node = TransportRobotService()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.mqtt_sub.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.mqtt_sub.mqttsubscriber.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
