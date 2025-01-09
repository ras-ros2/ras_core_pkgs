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
# from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from ras_common.transport.TransportWrapper import TransportMQTTPublisher
from aruco_interfaces.msg import ArucoMarkers
from rclpy_message_converter import json_message_converter



class ArmLogger(LifecycleNode):
    def __init__(self):
        super().__init__("arm_logger")

        self.get_logger().info('NODE STARTED')

        self.mqtt_pub = TransportMQTTPublisher("last/will/topic")

        joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        log_srv = self.create_service(StatusLog, '/traj_status', self.status_callback)
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)
        
        self.connect_to_aws()

        self.joint_status = JointState()
        self.trajlog = {}

    def connect_to_aws(self):
        self.mqtt_pub.connect_with_retries()
    
    def aruco_callback(self, msg):
        self.aruco_data = json_message_converter.convert_ros_message_to_json(msg)

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
    
    # def publish_with_retry(self, payload, delay=2):
    #     while True:
    #         try:
    #             chunk_size = 128 * 1024
    #             chunks = [payload[i:i + chunk_size] for i in range(0, len(payload), chunk_size)]
    #             for i, chunk in enumerate(chunks):
    #                 self.mqtt_client.publish(topic, chunk, 1)
    #                 self.get_logger().info("Message published successfully")
    #             break
    #         except Exception as e:
    #             self.get_logger().error(f"Publish failed: {e}. Retrying in {delay} seconds...")
    #             time.sleep(delay)
    #             self.connect_to_aws()
    
    def status_callback(self, request, response):
        self.trajlog = {
            "joint_state" : self.joint_list,
            "traj_status" : request.traj_status,
            "gripper_status" : request.gripper_status,
            "current_traj" : request.current_traj,
            "aruco_markers" : self.aruco_data
        }
        
        payload = json.dumps(self.trajlog)
        self.mqtt_pub.publish(payload)
        
        response.success = True

        return response

def main():
    rclpy.init(args=None)

    arm = ArmLogger()
    while rclpy.ok():
        rclpy.spin_once(arm, timeout_sec=0.1)
        arm.mqtt_pub.loop()
        
    arm.destroy_node()
    exit()

if __name__ == '__main__':
    main()
        


            