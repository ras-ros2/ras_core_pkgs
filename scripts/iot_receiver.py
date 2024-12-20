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
from ras_common.transport.TransportServer import TransportMQTTSubscriber
from rclpy.executors import MultiThreadedExecutor
import json
import yaml
import time
from ras_interfaces.action import BTInterface

class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')
        my_callback_group = ReentrantCallbackGroup()

        # self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_topic', 10)
        # self.service_sync = self.create_client(JointSat, "sync_arm", callback_group=my_callback_group)
        # self.fallback_client = self.create_client(LoadExp, "/fallback_info", callback_group=my_callback_group)
        self.path_client = self.create_client(SetPath, '/load_path')
        self._action_client = ActionClient(self,BTInterface,"bt_executor")

        self.instruction_msg = []

        self.instruction_flag = True
        self.mqtt_sub = TransportMQTTSubscriber("test/topic", self.custom_callback)
        # Connect to AWS IoT
        self.connect_to_aws()

        # Subscribe to the topic
        # self.mqtt_client.subscribe(cert_data["topic"], 1, self.custom_callback)
        # self.get_logger().info(f"Subscribed to topic: {cert_data['topic']}")
        self.payload = ''
    def connect_to_aws(self):
        """Attempt to connect to AWS IoT with retries"""
        while True:
            try:
                self.mqtt_sub.mqttsubscriber.connect()
                self.get_logger().info("Connected to AWS IoT")
                break
            except Exception as e:
                self.get_logger().error(f"Connection to AWS IoT failed: {e}. Retrying in 5 seconds...")
                time.sleep(5)

    def custom_callback(self, message):
        self.payload =  message.decode("utf-8")
        self.get_logger().info("Received Message")

        extract_path = os.path.join(os.environ['RAS_WORKSPACE_PATH'], "src", "ras_aws_transport", "real_bot_zip")

        print("Downloading the file using wget...")
        result = subprocess.run(
<<<<<<< Updated upstream
        ["cp", f"{os.environ['RAS_APP_PATH']}/serve/traj.zip", f"{extract_path}"],
        check=True,
        capture_output=True,
        text=True,
    )
        print("Download completed:")
=======
            ["cp", f"{os.environ['RAS_APP_PATH']}/serve/traj.zip", f"{extract_path}"],
            check=True,
            capture_output=True,
            text=True,
        )
        print("Download completed")
>>>>>>> Stashed changes

        extract_path = os.path.join(os.environ['RAS_WORKSPACE_PATH'], "src", "ras_aws_transport", "real_bot_zip")

        result2 = subprocess.run(
        ["unzip", "-o", f"{extract_path}/traj.zip", "-d", f"{extract_path}"],
        check=True,
        capture_output=True,
        text=True
        )

        path_zip = SetPath.Request()
        path_zip.path = extract_path
        self.path_client.call_async(path_zip)

        goal_msg = BTInterface.Goal()
        goal_msg.bt_path = path_zip.path + "/real.xml"
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    receiver = TrajectoryLogger()
    my_exec = MultiThreadedExecutor(num_threads=2)
    my_exec.add_node(receiver)
    try:
        while rclpy.ok():
            my_exec.spin_once(timeout_sec=0.1)
            receiver.mqtt_sub.mqttsubscriber.client.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        receiver.mqtt_sub.mqttsubscriber.disconnect()
        receiver.get_logger().info("Disconnected from AWS IoT")
        receiver.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
