#!/usr/bin/env python3

import os

path_for_config = os.path.join(os.environ["OSS_WORKSPACE_PATH"], "src", "oss_aws_transport", "aws_configs", "aws_config.json")
with open(path_for_config) as f:
    cert_data = json.load(f)

os.environ["AWS_ACCESS_KEY_ID"] = cert_data["AWS_ACCESS_KEY_ID"]
os.environ["AWS_SECRET_ACCESS_KEY"] = cert_data["AWS_SECRET_ACCESS_KEY"]

import rclpy
import json
from rclpy.node import Node
import boto3
from oss_interfaces.srv import SetPath
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


