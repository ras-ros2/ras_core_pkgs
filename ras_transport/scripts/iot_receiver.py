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
import signal

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
from ras_common.globals import RAS_APP_PATH, RAS_CONFIGS_PATH
from ras_transport.interfaces.TransportWrapper import TransportFileServer, TransportServiceServer
import json
import yaml
import time
from ras_bt_framework.managers.BaTMan import BaTMan
from ras_interfaces.msg import BTNodeStatus
from pathlib import Path
from ras_transport.interfaces.TransportWrapper import TransportFileClient
from ras_logging.ras_logger import RasLogger
import threading
import datetime

class TrajectoryLogger(LifecycleNode):
    def __init__(self):
        super().__init__('trajectory_logger')
        self.logger = RasLogger()
        my_callback_group = ReentrantCallbackGroup()

        self.path_client = self.create_client(SetPath, '/load_path')
        self.file_client = TransportFileClient()
        
        # Create client for the status logging service
        self.status_client = self.create_client(StatusLog, '/traj_status')

        self.instruction_msg = []

        self.instruction_flag = True
        self.remote_bt_server = TransportServiceServer("remote_bt", self.custom_callback)
        self.test_service_server = TransportServiceServer("test_service", self.test_service_callback)
        self.batman = BaTMan()

        # Connect to AWS IoT
        self.connect_to_aws()

        self.payload = ''

    def connect_to_aws(self):
        self.remote_bt_server.connect_with_retries()
        self.test_service_server.connect_with_retries()
        self.file_client.connect_with_retries()
        self.reciever_timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.remote_bt_server.loop()
        self.test_service_server.loop()

    def test_service_callback(self, message):
        try:
            print("TEST SERVICE CALLBACK RECEIVED MESSAGE")
            payload = message.decode("utf-8")
            payload_json = json.loads(payload)
            print(f"TEST SERVICE PAYLOAD: {payload_json}")
            self.logger.log_info(f"Received test service callback: {payload_json}")
            
            # Check if this is a command to start, stop, or get data from the ArUco calibration
            command = payload_json.get("command", "start")
            
            if command == "stop" and hasattr(self, 'aruco_process'):
                # This is a command to stop the ArUco calibration process
                return self.terminate_aruco_process()
            elif command == "get_data":
                # This is a command to get the latest calibration data without stopping
                calibration_data = self.get_latest_calibration_data()
                if calibration_data:
                    return json.dumps({
                        "success": True,
                        "calibration_data": calibration_data
                    })
                else:
                    return json.dumps({
                        "success": False,
                        "message": "No calibration data available"
                    })
            elif command == "start":
                # Run aruco_calibration.py when a start command is received
                try:
                    self.logger.log_info("Starting aruco_calibration.py process")
                    print("Starting aruco_calibration.py process")
                    # Run the process in the background with stdout/stderr redirected to files
                    # This prevents buffering issues that might cause the process to hang
                    stdout_file = open("/tmp/aruco_calibration_stdout.log", "w")
                    stderr_file = open("/tmp/aruco_calibration_stderr.log", "w")
                    
                    process = subprocess.Popen(
                        ["ros2", "run", "ras_perception", "aruco_calibration.py"],
                        stdout=stdout_file,
                        stderr=stderr_file,
                        # Don't capture output in pipes which can cause blocking
                        # Use files instead which are properly flushed
                        bufsize=1,
                        # Start in a new process group so it's not affected by signals sent to the parent
                        preexec_fn=os.setsid
                    )
                    
                    # Store the file handles to prevent them from being garbage collected
                    self.stdout_file = stdout_file
                    self.stderr_file = stderr_file
                    
                    # Store the process reference for later termination
                    self.aruco_process = process
                    self.logger.log_info("Stored aruco_calibration.py process reference for later termination")
                    print("ArUco calibration process started - it will run until explicitly terminated")
                    
                    self.logger.log_info("aruco_calibration.py process started")
                    return json.dumps({"success": True, "message": "Started aruco_calibration.py process"})
                except Exception as e:
                    self.logger.log_error(f"Error starting aruco_calibration.py: {e}")
                    print(f"Error starting aruco_calibration.py: {e}")
                    return json.dumps({"success": False, "error": str(e)})
            else:
                # Unknown command
                return json.dumps({"success": False, "message": f"Unknown command: {command}"})
        except Exception as e:
            print(f"TEST SERVICE ERROR: {e}")
            self.logger.log_error(f"Error in test service callback: {e}")
            return json.dumps({"success": False})
            
    def get_latest_calibration_data(self):
        """Get the latest calibration data from the temporary file written by aruco_calibration.py"""
        try:
            # Path to the temporary calibration file
            temp_calibration_file = os.path.join(RAS_CONFIGS_PATH, "temp_calibration.json")
            
            # Check if the file exists
            if os.path.exists(temp_calibration_file):
                # Read the calibration data from the file
                with open(temp_calibration_file, 'r') as f:
                    calibration_data = json.load(f)
                
                self.logger.log_info(f"Read calibration data from file: X={calibration_data['x']}cm, Y={calibration_data['y']}cm, Z={calibration_data['z']}cm")
                return calibration_data
            else:
                self.logger.log_warn("Temporary calibration file not found, using default values")
                # Return default values if file doesn't exist
                return {
                    "x": 39.0,  # Default X in cm
                    "y": 2.0,   # Default Y in cm
                    "z": 0.0,   # Default Z in cm
                    "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                }
            
        except Exception as e:
            self.logger.log_error(f"Error getting calibration data: {e}")
            # Return default values as fallback
            return {
                "x": 39.0,  # Default X in cm
                "y": 2.0,   # Default Y in cm
                "z": 0.0,   # Default Z in cm
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }

    def terminate_aruco_process(self):
        """Terminate the ArUco calibration process"""
        try:
            # First get the latest calibration data before terminating
            calibration_data = self.get_latest_calibration_data()
            
            if hasattr(self, 'aruco_process') and self.aruco_process is not None:
                self.logger.log_info("Terminating ArUco calibration process")
                print("Terminating ArUco calibration process")
                
                # Skip the ROS2 node kill approach since ROS2 doesn't have a direct 'kill' command
                # Instead, go directly to process termination
                
                # Terminate the process and its entire process group
                try:
                    # Since we started the process with os.setsid, we need to kill the entire process group
                    # to ensure all child processes are terminated
                    self.logger.log_info("Terminating ArUco calibration process group")
                    print("Terminating ArUco calibration process group")
                    
                    # Get the process group ID (same as process ID since we used os.setsid)
                    pgid = os.getpgid(self.aruco_process.pid)
                    
                    # Send SIGTERM to the entire process group
                    os.killpg(pgid, signal.SIGTERM)
                    
                    # Wait up to 3 seconds for graceful termination
                    for _ in range(6):
                        if self.aruco_process.poll() is not None:
                            break
                        time.sleep(0.5)
                    
                    # If still running, use SIGKILL as a last resort
                    if self.aruco_process.poll() is None:
                        self.logger.log_info("Process still running, sending SIGKILL")
                        print("Process still running, sending SIGKILL")
                        os.killpg(pgid, signal.SIGKILL)
                        time.sleep(1)
                    
                    # Check if the process is terminated
                    if self.aruco_process.poll() is not None:
                        self.logger.log_info(f"ArUco calibration process terminated with exit code {self.aruco_process.poll()}")
                        print(f"ArUco calibration process terminated with exit code {self.aruco_process.poll()}")
                    else:
                        self.logger.log_error("Failed to terminate ArUco calibration process")
                        print("Failed to terminate ArUco calibration process")
                except Exception as e:
                    self.logger.log_error(f"Error terminating process: {e}")
                    print(f"Error terminating process: {e}")
                
                # Clean up the file handles
                try:
                    # Close the stdout and stderr files if they exist
                    if hasattr(self, 'stdout_file') and self.stdout_file:
                        self.stdout_file.close()
                        self.stdout_file = None
                        self.logger.log_info("Closed stdout file")
                    
                    if hasattr(self, 'stderr_file') and self.stderr_file:
                        self.stderr_file.close()
                        self.stderr_file = None
                        self.logger.log_info("Closed stderr file")
                        print("Successfully killed remaining processes with pkill")
                    else:
                        self.logger.log_warn(f"pkill returned non-zero exit code: {pkill_process.returncode}")
                        print(f"pkill returned non-zero exit code: {pkill_process.returncode}")
                except Exception as e:
                    self.logger.log_error(f"Error using pkill: {e}")
                    print(f"Error using pkill: {e}")
                
                self.aruco_process = None
                if hasattr(self, 'stdout_file'):
                    self.stdout_file = None
                if hasattr(self, 'stderr_file'):
                    self.stderr_file = None
                self.logger.log_info("ArUco process reference and file handles reset")
                print("ArUco process reference and file handles reset")
                
                # Create the response with calibration data
                response = {"success": True, "message": "ArUco calibration process terminated"}
                if calibration_data:
                    response["calibration_data"] = calibration_data
                    self.logger.log_info(f"Sending calibration data: X={calibration_data['x']}cm, Y={calibration_data['y']}cm, Z={calibration_data['z']}cm")
                    print(f"Sending calibration data: X={calibration_data['x']}cm, Y={calibration_data['y']}cm, Z={calibration_data['z']}cm")
                else:
                    self.logger.log_warn("No calibration data available to send")
                    print("No calibration data available to send")
                
                return json.dumps(response)
            else:
                self.logger.log_warn("No ArUco calibration process to terminate")
                print("No ArUco calibration process to terminate")
                return json.dumps({"success": False, "message": "No ArUco calibration process to terminate"})
        except Exception as e:
            self.logger.log_error(f"Error in terminate_aruco_process: {e}")
            print(f"Error in terminate_aruco_process: {e}")
            return json.dumps({"success": False, "error": str(e)})

    def custom_callback(self, message):
        self.payload =  message.decode("utf-8")
        self.logger.log_info("Received Message")

        # Extract image filename from payload (if present)
        try:
            payload_json = json.loads(self.payload)
            image_filename = payload_json.get("image_filename")
            # Image download logic removed; handled in log_receiver instead
        except Exception as e:
            self.logger.log_warn(f"Could not parse payload for image filename: {e}")

        pkg_path = get_cmake_python_pkg_source_dir("ras_transport")
        if not pkg_path:
            self.logger.log_error("Unable to find the package path")
            status = False
            payload = json.dumps({"status": status}) 
            return  payload
        
        extract_path = os.path.join(str(pkg_path), "real_bot_zip")
        Path(extract_path).mkdir(parents=True, exist_ok=True)

        self.logger.log_info("Downloading the file using http...")
        result = self.file_client.download("xml_directory.zip", f"{extract_path}/xml_directory.zip")
        # result = subprocess.run(
        #     ["cp", f"{TransportFileServer.serve_path}/xml_directory.zip", f"{extract_path}"],
        #     check=True,
        #     capture_output=True,
        #     text=True,
        # )

        if result:
            self.logger.log_info("Download completed")
        else:
            self.logger.log_error("Download failed")
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
            self.logger.log_info("Behavior Tree Execution Successful")
            status = True
            
            # Call the status logging service
            self.call_status_log_service('SUCCESS')
        else:
            self.logger.log_info("Behavior Tree Execution Failed")
            status = False
        payload = json.dumps({"status": status}) 
        return payload
        
    def call_status_log_service(self, status_value):
        # Wait for service to be available
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            self.logger.log_warn('Status logging service not available')
            return
            
        # Create and send request
        request = StatusLog.Request()
        request.traj_status = status_value
        # request.current_traj = 0  # Default value if not provided
        request.gripper_status = False  # Default value if not provided
        
        future = self.status_client.call_async(request)
        future.add_done_callback(self.status_log_callback)
        
    def status_log_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.logger.log_info('Status log service call successful')
            else:
                self.logger.log_warn('Status log service call failed')
        except Exception as e:
            self.logger.log_error(f'Service call failed: {e}', e)

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
        receiver.logger.log_info("Disconnected from AWS IoT")
        receiver.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()