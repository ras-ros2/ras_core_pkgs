#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import json
import time
from oss_interfaces.action import ExecuteExp
from std_srvs.srv import SetBool
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from oss_interfaces.msg import Instruction
import ast
from trajectory_msgs.msg import JointTrajectory
from awscrt import mqtt 
import json
from connection_helper import ConnectionHelper
from oss_interfaces.srv import SetPath
from oss_interfaces.action import ExecuteExp

import os
import zipfile



class LinkHandler(Node):

    def __init__(self):
        super().__init__('link_sender')

        my_callback_group = ReentrantCallbackGroup()

        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)
        self.client = self.create_client(SetPath, "/send_file", callback_group=my_callback_group)
        self.send_client = ActionServer(self, ExecuteExp, "/execute_exp", self.send_callback, callback_group=my_callback_group)

        self.ws_path = os.environ["OSS_WORKSPACE_PATH"]
        self.path_for_config = os.path.join(self.ws_path, "src", "oss_aws_transport", "aws_configs", "iot_sender_config.json")
        discover_endpoints = False
        self.connection_helper = ConnectionHelper(self.get_logger(), self.path_for_config, discover_endpoints)
        

    def send_callback(self, goal_handle):
        self.get_logger().info("Starting Real Arm.....")
        zip_file_path = self.zip_xml_directory()
        path = os.path.join(self.ws_path, "src", "oss_bt_framework", "xml", "xml_directory.zip")
        self.send_zip_file_path(path)
        result = ExecuteExp.Result()
        result.success = True
        goal_handle.succeed()
        return result
        
    def send_zip_file_path(self, zip_file_path):
        request = SetPath.Request()
        request.path = zip_file_path
        
        self.client.wait_for_service()
        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(response.link)
        self.publish_with_retry(response.link)

    def zip_xml_directory(self):
    # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Define the path to the xml directory
        xml_dir_path = "/oss_sim_lab/ros2_ws/src/oss_bt_framework/xml/"
        
        # Define the path for the output zip file
        zip_file_path = "/oss_sim_lab/ros2_ws/src/oss_bt_framework/xml/xml_directory.zip"
        
        # Create a zip file and add all files in the xml directory to it
        with zipfile.ZipFile(zip_file_path, 'w') as zipf:
            for root, dirs, files in os.walk(xml_dir_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, start=xml_dir_path)
                    zipf.write(file_path, arcname)
        
        return zip_file_path

    def publish_with_retry(self, payload, delay=2):
        self.get_logger().info("Publishing to AWS IoT")
        self.connection_helper.mqtt_conn.publish(
            topic="test/topic",
            payload=payload,
            qos=mqtt.QoS.AT_LEAST_ONCE
        )

        time.sleep(0.25)

def main(args=None):
    rclpy.init(args=args)
    node = LinkHandler()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.destroy_node()
        node.get_logger().info("Disconnected from AWS IoT")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
