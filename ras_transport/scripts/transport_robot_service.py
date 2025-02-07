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
from ras_transport.interfaces.TransportWrapper import TransportMQTTSubscriber, TransportServiceServer
from ras_common.config.loaders.lab_setup import LabSetup
from ras_interfaces.srv import JointReq
from typing import Dict
from enum import Enum
from std_srvs.srv import SetBool
import json

class TransportCommands(Enum):
    HOME = "home"
    SYNC = "sync"

class TransportRobotService(Node):
    def __init__(self):
        super().__init__('transport_robot_service')
        self.server_transport_server = TransportServiceServer("server_transport", self.execute_request)
        self.connect_aws()

    def connect_aws(self):
        self.server_transport_server.connect_with_retries()

    def execute_request(self, message):
        payload = message.decode("utf-8")
        if payload == TransportCommands.HOME.value:
            self.get_logger().info("Executing Home command")
            req : JointReq.Request = JointReq.Request()
            LabSetup.init()
            home_joint_state: Dict[str, float|int] = LabSetup.conf.robot.home_joint_state
            req.joints.header.stamp = self.get_clock().now().to_msg()
            req.joints.name = list(home_joint_state.keys())
            req.joints.position = [float(value) for value in home_joint_state.values()]
            self.get_logger().info(f"Home joint state: {home_joint_state}")
            client = self.create_client(JointReq, '/move_to_joint_states')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response: JointReq.Response = future.result()
                if response.success:
                    self.get_logger().info("Home position reached successfully")
                    self.execute_request(TransportCommands.SYNC.value.encode("utf-8"))
                else:
                    self.get_logger().info("Home position doesn't reached")
            else:
                self.get_logger().error(f"Service call failed: {future.exception()}")
        elif payload == TransportCommands.SYNC.value:
            self.get_logger().info("Executing Sync command")
            req : SetBool.Request = SetBool.Request()
            req.data = True
            client = self.create_client(SetBool, '/dummy_logging_server')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service dummy_logging_server not available, waiting again...')
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response: SetBool.Response = future.result()
                if response.success:
                    self.get_logger().info("Sync command executed successfully")
                else:
                    self.get_logger().info("Sync command execution failed")
            else:
                self.get_logger().error(f"Service call failed: {future.exception()}")
        else:
            self.get_logger().info("Invalid command")

        payload = {
            "success": True
        }
        return json.dumps(payload)

def main(args=None):
    rclpy.init(args=args)
    node = TransportRobotService()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.server_transport_server.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.server_transport_server.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
