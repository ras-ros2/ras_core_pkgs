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
from sensor_msgs.msg import JointState
from ras_interfaces.srv import StatusLog
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from std_srvs.srv import Empty


# AWS IoT Parameters
host = "a1u5bxro5znv23-ats.iot.ap-south-1.amazonaws.com"
root_ca_path = "/workspaces/robo_arm_ws/src/xarm_cpp_custom/AWS_Cert/AmazonRootCA1.pem"
certificate_path = "/workspaces/robo_arm_ws/src/xarm_cpp_custom/AWS_Cert/4f163fd4aedc5fb7dac84fb40685a69859a5eec8b47351d0cbc7e393e1101588-certificate.pem.crt"
private_key_path = "/workspaces/robo_arm_ws/src/xarm_cpp_custom/AWS_Cert/4f163fd4aedc5fb7dac84fb40685a69859a5eec8b47351d0cbc7e393e1101588-private.pem.key"
port = 8883
topic = "reset/topic"
client_id = "senderClient"

class ArmHardReset(Node):
    def __init__(self):
        super().__init__("hard_reset")

        self.get_logger().info('NODE STARTED')

        self.mqtt_client = AWSIoTMQTTClient(client_id)
        self.mqtt_client.configureEndpoint(host, port)
        self.mqtt_client.configureCredentials(root_ca_path, private_key_path, certificate_path)
        self.mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline publish queueing
        self.mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.mqtt_client.configureConnectDisconnectTimeout(10)  # 10 sec
        self.mqtt_client.configureMQTTOperationTimeout(10)  # Increased to 10 sec
        self.mqtt_client.configureLastWill("last/will/topic", "Client disconnected unexpectedly", 1)

        self.create_service(Empty, "/hard_reset", self.hard_reset_callback)
        
        self.connect_to_aws()

    def hard_reset_callback(self, req, rep):
        self.get_logger().info("reset_called")
        payload = "reset"
        self.publish_with_retry(payload)

        return rep

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

def main():
    rclpy.init(args=None)

    arm = ArmHardReset()
    rclpy.spin(arm)
    arm.destroy_node()
    exit()

if __name__ == '__main__':
    main()
        


            
