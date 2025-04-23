#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import json
import time
from ras_interfaces.action import ExecuteExp
from std_srvs.srv import SetBool
from rclpy.action import ActionServer
# from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from ras_interfaces.msg import Instruction
import ast
from trajectory_msgs.msg import JointTrajectory
# from awscrt import mqtt 
import json
# from connection_helper import ConnectionHelper
from ras_interfaces.srv import SetPath
from ras_interfaces.action import ExecuteExp
from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher, TransportMQTTSubscriber, TransportServiceClient
from ras_transport.interfaces.TransportWrapper import TransportFileClient
import os
import zipfile
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_logging.ras_logger import RasLogger

# Set APP_TYPE
APP_TYPE = os.environ.get("APP_TYPE", "server")  # fallback default
if APP_TYPE == "server":
    from rclpy.action.server import ServerGoalHandle

class LinkHandler(Node):

    def __init__(self):
        super().__init__('link_sender')
        self.logger = RasLogger()

        my_callback_group = ReentrantCallbackGroup()

        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)
        self.file_client = TransportFileClient()
        self.send_client = ActionServer(self, ExecuteExp, "/execute_exp", self.send_callback, callback_group=my_callback_group)

        self.mqtt_sub_response_flag = False
        self.mqtt_sub_msg = None
        self.remote_bt_client = TransportServiceClient("remote_bt")
        self.connect_to_aws()
        
    def connect_to_aws(self):
        self.remote_bt_client.connect_with_retries()
        self.file_client.connect_with_retries()

    # def send_callback(self, goal_handle: ServerGoalHandle):
    #     self.get_logger().info("Starting Real Arm.....")
    #     zip_file_path = self.zip_xml_directory()
    #     result = ExecuteExp.Result()
    #     if zip_file_path == "":
    #         self.get_logger().error("Zip file not created")
    #         result.success = False
    #         goal_handle.abort()
    #     else:
    #         resp: str = self.send_zip_file_path(zip_file_path)
    #         data = json.loads(resp)
    #         status = data.get("status")
    #         result.success = status
    #         goal_handle.succeed()
    #     return result


    def send_callback(self, goal_handle):
        self.logger.log_info("Starting Real Arm.....")

        if APP_TYPE == "server":
            zip_file_path = self.zip_xml_directory()
        else:  # robot
            pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
            zip_file_path = os.path.join(str(pkg_path), "xml", "xml_directory.zip")

        result = ExecuteExp.Result()

        if not zip_file_path or not os.path.exists(zip_file_path):
            self.logger.log_error("Zip file not found or created")
            result.success = False
            if APP_TYPE == "server":
                goal_handle.abort()
        else:
            resp: str = self.send_zip_file_path(zip_file_path)
            data = json.loads(resp)
            result.success = data.get("status")
            goal_handle.succeed()

        return result


    def send_zip_file_path(self, zip_file_path) -> str:
        request = SetPath.Request()
        request.path = zip_file_path

        result = self.file_client.upload(zip_file_path, "xml_directory.zip")
        if result:
            return self.remote_bt_client.call("xml_directory.zip")
        else:
            return json.dumps({"status": False})

    # def zip_xml_directory(self):
    #     pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
    #     if pkg_path is None:
    #         return ""
    #     else:
    #         xml_dir_path = os.path.join(pkg_path, "xml")
    #         zip_dir_path = os.path.join(pkg_path, "zip")
    #         if not os.path.exists(zip_dir_path):
    #             os.makedirs(zip_dir_path)
    #         zip_file_path = zip_dir_path+"/xml_directory.zip"
    #         if os.path.exists(xml_dir_path+"/xml_directory.zip"):
    #             # Remove the existing zip file from previous bugs
    #             os.remove(xml_dir_path+"/xml_directory.zip")
    #         with zipfile.ZipFile(zip_file_path, 'w') as zipf:
    #             for root, dirs, files in os.walk(xml_dir_path):
    #                 for file in files:
    #                     file_path = os.path.join(root, file)
    #                     arcname = os.path.relpath(file_path, start=xml_dir_path)
    #                     zipf.write(file_path, arcname)
    #         return zip_file_path

    def zip_xml_directory(self):
        pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")

        if pkg_path is None:
            self.logger.log_error("Could not find the package path for ras_bt_framework")
            return ""

        xml_dir_path = os.path.join(pkg_path, "xml")
        
        # Server stores zip in a separate "zip" folder; Robot stores it inside the xml folder
        if APP_TYPE == "server":
            zip_dir_path = os.path.join(pkg_path, "zip")
            if not os.path.exists(zip_dir_path):
                os.makedirs(zip_dir_path)
            zip_file_path = os.path.join(zip_dir_path, "xml_directory.zip")
        else:  # robot
            zip_file_path = os.path.join(xml_dir_path, "xml_directory.zip")

        # Clean up any existing zip file (especially important on server)
        if os.path.exists(zip_file_path):
            os.remove(zip_file_path)

        # Create the new zip archive
        with zipfile.ZipFile(zip_file_path, 'w') as zipf:
            for root, dirs, files in os.walk(xml_dir_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, start=xml_dir_path)
                    zipf.write(file_path, arcname)

        return zip_file_path

    


    

def main(args=None):
    rclpy.init(args=args)
    node = LinkHandler()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.remote_bt_client.loop()

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.destroy_node()
        node.logger.log_info("Disconnected from AWS IoT")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

