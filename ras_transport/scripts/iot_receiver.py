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
from ras_interfaces.srv import JointSat, LoadExp, StatusLog
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
        
        # Create client for the status logging service
        self.status_client = self.create_client(StatusLog, '/traj_status')

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

        # Extract image filename from payload (if present)
        try:
            payload_json = json.loads(self.payload)
            image_filename = payload_json.get("image_filename")
            # Image download logic removed; handled in log_receiver instead
        except Exception as e:
            self.get_logger().warn(f"Could not parse payload for image filename: {e}")

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
            
            # Call the status logging service
            self.call_status_log_service('SUCCESS')
        else:
            self.get_logger().info("Behavior Tree Execution Failed")
            status = False
        payload = json.dumps({"status": status}) 
        return payload
        
    def call_status_log_service(self, status_value):
        # Wait for service to be available
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Status logging service not available')
            return
            
        # Create and send request
        request = StatusLog.Request()
        request.traj_status = status_value
        request.current_traj = 0  # Default value if not provided
        request.gripper_status = False  # Default value if not provided
        
        future = self.status_client.call_async(request)
        future.add_done_callback(self.status_log_callback)
        
    def status_log_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Status log service call successful')
            else:
                self.get_logger().warn('Status log service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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
    main()#!/usr/bin/env python3

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
from ras_interfaces.srv import JointSat, LoadExp, StatusLog
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
        
        # Create client for the status logging service
        self.status_client = self.create_client(StatusLog, '/traj_status')

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

        # Extract image filename from payload (if present)
        try:
            payload_json = json.loads(self.payload)
            image_filename = payload_json.get("image_filename")
            # Image download logic removed; handled in log_receiver instead
        except Exception as e:
            self.get_logger().warn(f"Could not parse payload for image filename: {e}")

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
            
            # Call the status logging service
            self.call_status_log_service('SUCCESS')
        else:
            self.get_logger().info("Behavior Tree Execution Failed")
            status = False
        payload = json.dumps({"status": status}) 
        return payload
        
    def call_status_log_service(self, status_value):
        # Wait for service to be available
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Status logging service not available')
            return
            
        # Create and send request
        request = StatusLog.Request()
        request.traj_status = status_value
        request.current_traj = 0  # Default value if not provided
        request.gripper_status = False  # Default value if not provided
        
        future = self.status_client.call_async(request)
        future.add_done_callback(self.status_log_callback)
        
    def status_log_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Status log service call successful')
            else:
                self.get_logger().warn('Status log service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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
    main()#!/usr/bin/env python3

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
from ras_interfaces.srv import JointSat, LoadExp, StatusLog
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
        
        # Create client for the status logging service
        self.status_client = self.create_client(StatusLog, '/traj_status')

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

        # Extract image filename from payload (if present)
        try:
            payload_json = json.loads(self.payload)
            image_filename = payload_json.get("image_filename")
            # Image download logic removed; handled in log_receiver instead
        except Exception as e:
            self.get_logger().warn(f"Could not parse payload for image filename: {e}")

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
            
            # Call the status logging service
            self.call_status_log_service('SUCCESS')
        else:
            self.get_logger().info("Behavior Tree Execution Failed")
            status = False
        payload = json.dumps({"status": status}) 
        return payload
        
    def call_status_log_service(self, status_value):
        # Wait for service to be available
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Status logging service not available')
            return
            
        # Create and send request
        request = StatusLog.Request()
        request.traj_status = status_value
        request.current_traj = 0  # Default value if not provided
        request.gripper_status = False  # Default value if not provided
        
        future = self.status_client.call_async(request)
        future.add_done_callback(self.status_log_callback)
        
    def status_log_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Status log service call successful')
            else:
                self.get_logger().warn('Status log service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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