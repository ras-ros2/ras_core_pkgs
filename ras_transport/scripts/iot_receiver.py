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
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ras_interfaces.srv import JointSat, LoadExp
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from ras_interfaces.srv import SetPath
from rclpy.executors import MultiThreadedExecutor
from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher, TransportMQTTSubscriber
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_common.globals import RAS_APP_PATH
from ras_transport.interfaces.TransportWrapper import TransportFileServer, TransportServiceServer
import json
import yaml
import time
from ras_bt_framework.managers.BaTMan import BaTMan
from ras_interfaces.msg import BTNodeStatus
from pathlib import Path
from ras_transport.interfaces.TransportWrapper import TransportFileClient

class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')
        my_callback_group = ReentrantCallbackGroup()

        self.path_client = self.create_client(SetPath, '/load_path')
        self.file_client = TransportFileClient()

        self.instruction_msg = []

        self.instruction_flag = True
        self.remote_bt_server = TransportServiceServer("remote_bt", self.custom_callback)

        self.batman = BaTMan()

        # Connect to AWS IoT
        self.connect_to_aws()

        self.payload = ''

    def connect_to_aws(self):
        self.remote_bt_server.connect_with_retries()
        self.file_client.connect_with_retries()
        self.reciever_timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.remote_bt_server.loop()

    def custom_callback(self, message):
        self.payload =  message.decode("utf-8")
        self.get_logger().info("Received Message")

        pkg_path = get_cmake_python_pkg_source_dir("ras_transport")
        if not pkg_path:
            self.get_logger().error("Unable to find the package path")
            status = False
            payload = json.dumps({"status": status}) 
            return  payload
        
        extract_path = os.path.join(str(pkg_path), "real_bot_zip")
        Path(extract_path).mkdir(parents=True, exist_ok=True)

        self.get_logger().info("Downloading the file using http...")
        result = self.file_client.download("xml_directory.zip", f"{extract_path}/xml_directory.zip")
        # result = subprocess.run(
        #     ["cp", f"{TransportFileServer.serve_path}/xml_directory.zip", f"{extract_path}"],
        #     check=True,
        #     capture_output=True,
        #     text=True,
        # )

        if result:
            self.get_logger().info("Download completed")
        else:
            self.get_logger().error("Download failed")
            payload = json.dumps({"status": False})
            return payload

        result2 = subprocess.run(
            ["unzip", "-o", f"{extract_path}/xml_directory.zip", "-d", f"{extract_path}"],
            check=True,
            capture_output=True,
            text=True
        )

        path_zip = SetPath.Request()
        path_zip.path = extract_path
        self.path_client.call_async(path_zip)

        bt_path = path_zip.path + "/real.xml"
        status = self.batman.execute_bt(bt_path)
        if status in [BTNodeStatus.SUCCESS, BTNodeStatus.IDLE]:
            self.get_logger().info("Behavior Tree Execution Successful")
            status = True
        else:
            self.get_logger().info("Behavior Tree Execution Failed")
            status = False
        payload = json.dumps({"status": status}) 
        return payload

def main(args=None):
    rclpy.init(args=args)
    receiver = TrajectoryLogger()
    my_exec = MultiThreadedExecutor(num_threads=8)
    my_exec.add_node(receiver)
    my_exec.add_node(receiver.batman)
    try:
        while rclpy.ok():
            my_exec.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        receiver.remote_bt_server.disconnect()
        receiver.get_logger().info("Disconnected from AWS IoT")
        receiver.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
