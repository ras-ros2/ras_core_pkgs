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

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ras_interfaces.srv import JointSat, LoadExp
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import json
import yaml
import time

class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')

        my_callback_group = ReentrantCallbackGroup()

        self.ws_path = os.environ["RAS_WORKSPACE_PATH"]

        self.path_for_config = os.path.join(self.ws_path, "src", "ras_aws_transport", "aws_configs", "log_receiver_config.json")

        with open(self.path_for_config) as f:
          cert_data = json.load(f)

        self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_topic', 10)
        self.service_sync = self.create_client(JointSat, "sync_arm", callback_group=my_callback_group)
        self.fallback_client = self.create_client(LoadExp, "/fallback_info", callback_group=my_callback_group)
        # Initialize the MQTT client
        self.mqtt_client = AWSIoTMQTTClient(cert_data["clientID"])
        self.mqtt_client.configureEndpoint(cert_data["endpoint"], cert_data["port"])
        self.mqtt_client.configureCredentials(cert_data["rootCAPath"], cert_data["privateKeyPath"], cert_data["certificatePath"])
        self.mqtt_client.configureOfflinePublishQueueing(-1)  # Infinite offline publish queueing
        self.mqtt_client.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.mqtt_client.configureConnectDisconnectTimeout(10.0)  # 10 sec
        self.mqtt_client.configureMQTTOperationTimeout(10.0)  # 10 sec
        self.mqtt_client.configureLastWill("last/will/topic", "Client disconnected unexpectedly", 1)
        self.instruction_msg = []

        self.instruction_flag = True

        # Connect to AWS IoT
        self.connect_to_aws()

        # Subscribe to the topic
        self.mqtt_client.subscribe(cert_data["topic"], 1, self.custom_callback)
        self.get_logger().info(f"Subscribed to topic: {cert_data['topic']}")
        self.payload = ''
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

    def custom_callback(self, client, userdata, message):
        self.payload =  message.payload.decode("utf-8")
        self.get_logger().info("Received Message")

        if not self.payload:
            self.get_logger().info("Received an empty payload.")
            return

        try:
            log_data = json.loads(self.payload)
            with open("log.txt", 'a') as file:
                yaml.dump(log_data, file)
            if log_data["traj_status"] == "SUCCESS":
                print("SUCCESS")
                request = JointSat.Request()
                request.joint_state.position = log_data["joint_state"]
                self.future = self.service_sync.call_async(request)
                rclpy.spin_until_future_complete(self, self.future)


            if log_data["traj_status"] == "FAILED":
                print("FAILED")
                # request = SetBool.Request()
                # request.data = True
                # self.future = self.service_.call_async(request)
                request = LoadExp.Request()
                request.instruction_no = str(log_data["current_traj"])
                request.picked_object = "beaker-1"
                self.future2 = self.fallback_client.call_async(request)
                rclpy.spin_until_future_complete(self, self.future2)


                

        except json.JSONDecodeError as e:
            pass
            # self.get_logger().error(f"JSONDecodeError: {e}")
        except KeyError as e:
            pass
            # self.get_logger().error(f"KeyError: {e}")
        except Exception as e:
            pass
            # self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    receiver = TrajectoryLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(receiver)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        receiver.destroy_node()
        receiver.mqtt_client.disconnect()
        receiver.get_logger().info("Disconnected from AWS IoT")
        rclpy.shutdown()

if __name__ == '__main__':
    main()