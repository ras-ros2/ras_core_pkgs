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
from ras_transport.interfaces.TransportWrapper import TransportMQTTSubscriber
from ras_transport.interfaces.TransportWrapper import TransportFileClient
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_common.globals import RAS_APP_PATH
from geometry_msgs.msg import Pose, Point, Quaternion
import os
import json
import yaml
from pathlib import Path
import time


class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')

        my_callback_group = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(JointTrajectory, 'trajectory_topic', 10)
        self.service_sync = self.create_client(JointSat, "sync_arm", callback_group=my_callback_group)
        self.fallback_client = self.create_client(LoadExp, "/fallback_info", callback_group=my_callback_group)
        # Initialize the MQTT client
        self.aruco_client = self.create_client(ArucoPoses, "/aruco_poses", callback_group=my_callback_group)
        self.instruction_msg = []
        self.file_client = TransportFileClient()
        
        # Ensure log directories exist
        logs_path = Path(RAS_APP_PATH) / "logs"
        logs_path.mkdir(parents=True, exist_ok=True)
        (logs_path / "images").mkdir(parents=True, exist_ok=True)
        
        self.mqtt_sub = TransportMQTTSubscriber("last/will/topic", self.custom_callback)
        self.instruction_flag = True
        self.aruco_sync_flag = True

        # Connect to AWS IoT
        self.connect_to_aws()

        self.payload = ''

    def connect_to_aws(self):
        self.mqtt_sub.connect_with_retries()

    def custom_callback(self, message):
        print(f"message is {message} and type is {type(message)}")
        self.payload =  message.decode("utf-8")
        print(f"now type is {type(self.payload)}")
        self.get_logger().info("Received Message")

        if not self.payload:
            self.get_logger().info("Received an empty payload.")
            return

        try:
            print(f"payload: {self.payload[:200]}...")  # Print first 200 chars to avoid flooding logs
            log_data = json.loads(self.payload)
            print(f"Log data type: {log_data.get('type')}")

            # Extract and print image filename for IoT receiver
            pkg_path = get_cmake_python_pkg_source_dir("ras_app_main")
            extract_path = os.path.join(str(pkg_path), "../../../image_logs")
            Path(extract_path).mkdir(parents=True, exist_ok=True)
            image_filename = log_data.get("image_filename")
            if image_filename:
                self.get_logger().info(f"Received image filename from MQTT: {image_filename}")
                # Download the image file using the file client
                img_result = self.file_client.download(image_filename, f"{extract_path}/{image_filename}")
                if img_result:
                    self.get_logger().info(f"Successfully downloaded image: {extract_path}/{image_filename}")
                else:
                    self.get_logger().warn(f"Failed to download image: {image_filename}")
            else:
                self.get_logger().warn("No image filename found in this message.")

            # Handle image type specially
            if log_data.get('type') == 'image' and 'image_base64' in log_data:
                print("Found image type with image_base64, handling image")
                self._handle_image_log(log_data)
            else:
                # Regular log entry - save to log.txt
                log_path = Path(RAS_APP_PATH) / "logs" / "log.txt"
                log_path.parent.mkdir(parents=True, exist_ok=True)
                with log_path.open("a") as file:
                    yaml.dump(log_data, file)

            if self.aruco_sync_flag == False:
                aruco_pose = ArucoPoses.Request()
                poses_list = json.loads(log_data["aruco_markers"])["poses"]
                marker_id_list = json.loads(log_data["aruco_markers"])["marker_ids"]

                aruco_pose.marker_id_list = marker_id_list

                aruco_pose.poses = [
                    Pose(
                        position=Point(x=pose["position"]["x"], y=pose["position"]["y"], z=pose["position"]["z"]),
                        orientation=Quaternion(x=pose["orientation"]["x"], y=pose["orientation"]["y"], z=pose["orientation"]["z"], w=pose["orientation"]["w"])
                    ) for pose in poses_list
                ]
                
                self.aruco_client.call_async(aruco_pose)
                self.get_logger().info("Spawing Started...")
                # self.aruco_sync_flag = False
                
            if log_data.get("traj_status") == "SUCCESS":
                print("SUCCESS")
                request = JointSat.Request()
                request.joint_state.position = log_data["joint_state"]
                self.future = self.service_sync.call_async(request)
                rclpy.spin_until_future_complete(self, self.future)

            if log_data.get("traj_status") == "FAILED":
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
            
    def _handle_image_log(self, log_data):
        """Extract and save image data from log entries with type 'image'"""
        try:
            import base64
            
            # Create images directory if it doesn't exist
            images_dir = Path(RAS_APP_PATH) / "logs" / "images"
            images_dir.mkdir(parents=True, exist_ok=True)
            
            print(f"Handling image data with keys: {list(log_data.keys())}")
            if 'image_base64' not in log_data:
                print(f"Warning: No image_base64 found in log_data, skipping image processing")
                return
                
            # Extract metadata
            timestamp = log_data.get('timestamp', self._get_timestamp())
            description = log_data.get('description', 'image')
            ext = 'jpg'  # Default extension
            
            # Determine file extension from path if available
            if 'path' in log_data:
                path = log_data['path']
                print(f"Image path: {path}")
                if '.' in path:
                    ext = path.split('.')[-1]
            
            # Create a unique filename
            image_path = images_dir / f"img_{timestamp}_{description.replace(' ', '_')}.{ext}"
            print(f"Will save image to: {image_path}")
            
            # Decode base64 image and save to file
            image_data = base64.b64decode(log_data['image_base64'])
            print(f"Decoded image data length: {len(image_data)} bytes")
            with open(image_path, 'wb') as f:
                f.write(image_data)
                
            self.get_logger().info(f"Saved image to: {image_path}")
            
            # Also log metadata to log.txt
            log_meta = {
                'type': 'image_metadata',
                'timestamp': timestamp,
                'description': description,
                'saved_path': image_path.absolute().as_uri()
            }
            
            log_path = Path(RAS_APP_PATH) / "logs" / "log.txt"
            with log_path.open("a") as file:
                yaml.dump(log_meta, file)
                
        except Exception as e:
            import traceback
            print(f"Error in _handle_image_log: {e}")
            print(traceback.format_exc())
            self.get_logger().error(f"Failed to save image: {e}")
    
    def _get_timestamp(self):
        """Generate a timestamp string"""
        from datetime import datetime
        return datetime.now().strftime('%Y%m%d_%H%M%S')

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