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
import re
import time
import yaml
import json
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from sensor_msgs.msg import JointState
from ras_interfaces.srv import StatusLog
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient


# AWS IoT Parameters
host = "a1u5bxro5znv23-ats.iot.ap-south-1.amazonaws.com"
root_ca_path = "/ras_real_lab/ros2_ws/src/ras_aws_transport/aws_cert/AmazonRootCA1.pem"
certificate_path = "/ras_real_lab/ros2_ws/src/ras_aws_transport/aws_cert/4f163fd4aedc5fb7dac84fb40685a69859a5eec8b47351d0cbc7e393e1101588-certificate.pem.crt"
private_key_path = "/ras_real_lab/ros2_ws/src/ras_aws_transport/aws_cert/4f163fd4aedc5fb7dac84fb40685a69859a5eec8b47351d0cbc7e393e1101588-private.pem.key"
port = 8883
topic = "logging/topic"
client_id = "log_sender_client"

class ArmLogger(LifecycleNode):
    def __init__(self):
        super().__init__("arm_logger")

        self.get_logger().info('NODE STARTED')

        self.ws_path = os.environ["RAS_WORKSPACE_PATH"]

        self.path_for_config = os.path.join(self.ws_path, "src", "ras_aws_transport", "aws_configs", "log_sender_config.json")

        with open(self.path_for_config) as f:
          cert_data = json.load(f)

        self.mqtt_client = AWSIoTMQTTClient(cert_data["clientID"])
        self.mqtt_client.configureEndpoint(cert_data["endpoint"], cert_data["port"])
        self.mqtt_client.configureCredentials(cert_data["rootCAPath"], cert_data["privateKeyPath"], cert_data["certificatePath"])
        self.mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline publish queueing
        self.mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.mqtt_client.configureConnectDisconnectTimeout(10.0)  # 10 sec
        self.mqtt_client.configureMQTTOperationTimeout(10.0)  # 10 sec
        self.mqtt_client.configureLastWill("last/will/topic", "Client disconnected unexpectedly", 1)

        joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        log_srv = self.create_service(StatusLog, '/traj_status', self.status_callback)

        self.connect_to_aws()

        self.joint_status = JointState()
        self.trajlog = {}

    def connect_to_aws(self):
        """Attempt to connect to AWS IoT with retries"""
        while True:
            try:
                self.mqtt_client.connect()
                self.get_logger().info("Connected to AWS IoT")
                break
            except Exception as e:
                self.get_logger().error(f"Connection to AWS IoT failed: {e}. Retrying in 5 seconds...")
                time.sleep(5)

    def get_next_log_filename(self, directory, prefix='log', extension='.txt'):
        files = os.listdir(directory)
        
        pattern = re.compile(rf'{prefix}(\d+){extension}')
        
        max_number = 0
        for file in files:
            match = pattern.match(file)
            if match:
                number = int(match.group(1))
                if number > max_number:
                    max_number = number
        
        next_number = max_number + 1
        next_filename = f"{prefix}{next_number}{extension}"
        
        return next_filename
    
    def joint_callback(self, msg):
        self.joint_list = []
        count = 1
        for i in range(0, len(msg.name)):
            for j in range(0, len(msg.name)):
                if str(count) in msg.name[j]:
                    self.joint_list.append(msg.position[j])
                    count = count + 1
    
    def publish_with_retry(self, payload, delay=2):
        while True:
            try:
                chunk_size = 128 * 1024
                chunks = [payload[i:i + chunk_size] for i in range(0, len(payload), chunk_size)]
                for i, chunk in enumerate(chunks):
                    self.mqtt_client.publish(topic, chunk, 1)
                    self.get_logger().info("Message published successfully")
                break
            except Exception as e:
                self.get_logger().error(f"Publish failed: {e}. Retrying in {delay} seconds...")
                time.sleep(delay)
                self.connect_to_aws()
    
    def status_callback(self, request, response):
        self.trajlog = {
            "joint_state" : self.joint_list,
            "traj_status" : request.traj_status,
            "gripper_status" : request.gripper_status,
            "current_traj" : request.current_traj
        }
        
        payload = json.dumps(self.trajlog)
        self.publish_with_retry(payload)
        
        response.success = True

        return response

def main():
    rclpy.init(args=None)

    arm = ArmLogger()
    rclpy.spin(arm)
    arm.destroy_node()
    exit()

if __name__ == '__main__':
    main()
        


            