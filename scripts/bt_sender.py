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
path_for_config = os.path.join(os.environ["RAS_WORKSPACE_PATH"], "src", "ras_aws_transport", "aws_configs", "aws_config.json")
with open(path_for_config) as f:
    cert_data = json.load(f)

os.environ["AWS_ACCESS_KEY_ID"] = cert_data["AWS_ACCESS_KEY_ID"]
os.environ["AWS_SECRET_ACCESS_KEY"] = cert_data["AWS_SECRET_ACCESS_KEY"]

import rclpy
from rclpy.node import Node
import boto3
from ras_interfaces.srv import SetPath
from botocore.config import Config

class BucketUpload(Node):
    def __init__(self):
        super().__init__("file_upload_node")
        self.get_logger().info("Node Initialized")
        self.create_service(SetPath, "/send_file", self.upload_callback)
        self.bucket_name = 'trajbucket'
        self.object_name = "traj.zip"
        self.s3_client = boto3.client('s3',region_name='ap-south-1', config=Config(signature_version='s3v4'))
        self.expiration = 3600
    
    def upload_callback(self, req, resp):

        file_path = req.path

        self.s3_client.upload_file(file_path, self.bucket_name, self.object_name)
        presigned_url = self.s3_client.generate_presigned_url(
        'get_object',
        Params={'Bucket': self.bucket_name, 'Key': self.object_name},
        ExpiresIn=self.expiration
        )
        self.get_logger().info(f"{self.object_name} Sucessfully Uploaded")

        resp.link = presigned_url
        return resp
    
def main():
    rclpy.init(args=None)
    node = BucketUpload()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


