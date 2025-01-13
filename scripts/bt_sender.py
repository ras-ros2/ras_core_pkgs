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

import os
import json
import rclpy
from rclpy.node import Node
# import boto3
from ras_interfaces.srv import SetPath
# from botocore.config import Config
from ras_common.transport.TransportWrapper import TransportFileClient


class BucketUpload(Node):
    def __init__(self):
        super().__init__("file_upload_node")
        self.get_logger().info("Node Initialized")
        self.create_service(SetPath, "/send_file", self.upload_callback)
        self.object_name = "traj.zip"
        self.file_client = TransportFileClient("real")
    
    def upload_callback(self, req, resp):
        file_path = req.path
        self.get_logger().info(f"{self.object_name} Sucessfully Uploaded")
        self.file_client.upload(file_path, self.object_name)
        resp.link = self.object_name
        return resp
    
def main():
    rclpy.init(args=None)
    node = BucketUpload()
    node.file_client.connect_with_retries()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


