#!/usr/bin/env python3

"""
Copyright (C) 2024 Harsh Davda

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
Harsh Davda
Email: info@opensciencestack.org
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from ras_interfaces.srv import GripperLog
from rclpy.callback_groups import ReentrantCallbackGroup
from ras_logging.ras_logger import RasLogger
from std_msgs.msg import Bool  # For publishing the gripper status as a Bool message

class GripperStatus(Node):
    def __init__(self):
        super().__init__("fake_gripper_node")
        self.logger = RasLogger()
        self.my_callback_group = ReentrantCallbackGroup()

        self.logger.log_info("Node Init")
        self.create_service(SetBool, "/fake_gripper", self.gripper_callback, callback_group=self.my_callback_group)
        self.gripper_client = self.create_client(GripperLog, "/gripper_status", callback_group=self.my_callback_group)

        # A publisher to publish gripper status
        self.gripper_status_publisher = self.create_publisher(Bool, '/gripper_status', 10)

    def gripper_callback(self, req, resp):

        gripper_data = GripperLog.Request()
        if req.data == True:
            self.logger.log_info("Gripper ON")
            gripper_data.gripper_status = True
        else:
            self.logger.log_info("Gripper OFF")
            gripper_data.gripper_status = False
        
        self.gripper_client.call_async(gripper_data)

        # Publish the gripper status to /gripper_status topic
        gripper_status_msg = Bool()
        gripper_status_msg.data = gripper_data.gripper_status
        self.gripper_status_publisher.publish(gripper_status_msg)

        resp.success = True

        return resp

def main():
    rclpy.init(args=None)
    node = GripperStatus()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

