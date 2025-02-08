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
from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher, TransportServiceClient, TransportServiceServer
from ras_interfaces.srv import ReadBlack
import time
from enum import Enum
import json

class TransportCommands(Enum):
    HOME = "home"
    SYNC = "sync"

class TransportServerService(Node):
    def __init__(self):
        super().__init__('transport_server_service')
        self.service_server = self.create_service(ReadBlack, '/server_transport', self.service_server_cb)
        self.server_transport_client = TransportServiceClient("server_transport")
        self.sync_robot_server = TransportServiceServer("sync_robot", self.sync_robot_cb)
        self.connect_aws()
    

    def sync_robot_cb(self, message):
        print("sync data received") # TODO(Sachin): remove this later
        payload = json.loads(message)
        if payload["sync_robot"]:
            print("Syncing robot")
            payload = {
                "success": True
            }
        else:
            print("Syncing is not performed")
            payload = {
                "success": True
            }
        return json.dumps(payload)

        

    

    def connect_aws(self):
        self.server_transport_client.connect_with_retries()
        self.sync_robot_server.connect_with_retries()

    def service_server_cb(self, request: ReadBlack.Request, response: ReadBlack.Response):
        if request.blackboard == TransportCommands.HOME.value:
            req = TransportCommands.HOME.value
        elif request.blackboard == TransportCommands.SYNC.value:
            req = TransportCommands.SYNC.value
        else:
            response.success = False
            return response
        res = self.server_transport_client.call(req)
        payload = json.loads(res)
        response.success = payload["success"]
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TransportServerService()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.server_transport_client.loop()
            node.sync_robot_server.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.server_transport_client.disconnect()
        node.sync_robot_server.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
