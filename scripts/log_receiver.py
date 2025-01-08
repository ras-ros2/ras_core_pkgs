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
from ras_interfaces.srv import JointSat, LoadExp, ArucoPoses
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration
# from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from rclpy.callback_groups import ReentrantCallbackGroup
from ras_common.transport.TransportWrapper import TransportMQTTSubscriber
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import json
import yaml
import time

class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')

        my_callback_group = ReentrantCallbackGroup()

        self.ws_path = os.environ["RAS_WORKSPACE_PATH"]


        self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_topic', 10)
        self.service_sync = self.create_client(JointSat, "sync_arm", callback_group=my_callback_group)
        self.fallback_client = self.create_client(LoadExp, "/fallback_info", callback_group=my_callback_group)
        # Initialize the MQTT client
        self.aruco_client = self.create_client(ArucoPoses, "/aruco_poses", callback_group=my_callback_group)
        self.instruction_msg = []
        self.mqtt_sub = TransportMQTTSubscriber("last/will/topic", self.custom_callback)
        self.instruction_flag = True
        self.aruco_sync_flag = True

        # Connect to AWS IoT
        self.connect_to_aws()

        self.payload = ''

    def connect_to_aws(self):
        self.mqtt_sub.connect_with_retries()

    def custom_callback(self, message):
        self.payload =  message.decode("utf-8")
        self.get_logger().info("Received Message")

        if not self.payload:
            self.get_logger().info("Received an empty payload.")
            return

        try:
            log_data = json.loads(self.payload)
            with open("/ras_sim_lab/logs/log.txt", 'a') as file:
                yaml.dump(log_data, file)

            if self.aruco_sync_flag == True:
                aruco_pose = ArucoPoses.Request()
                poses_list = json.loads(log_data["aruco_markers"])["poses"]
    
                aruco_pose.poses = [
                    Pose(
                        position=Point(x=pose["position"]["x"], y=pose["position"]["y"], z=pose["position"]["z"]),
                        orientation=Quaternion(x=pose["orientation"]["x"], y=pose["orientation"]["y"], z=pose["orientation"]["z"], w=pose["orientation"]["w"])
                    ) for pose in poses_list
                ]
                
                self.aruco_client.call_async(aruco_pose)
                self.get_logger().info("Spawing Started...")
                
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
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    receiver = TrajectoryLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(receiver,timeout_sec=0.1)
            receiver.mqtt_sub.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        receiver.destroy_node()
        receiver.mqtt_sub.disconnect()
        receiver.get_logger().info("Disconnected from AWS IoT")
        rclpy.shutdown()

if __name__ == '__main__':
    main()