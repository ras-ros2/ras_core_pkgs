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
import datetime
import time
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
from ras_common.globals import RAS_APP_PATH
from ras_transport.interfaces.TransportWrapper import TransportFileServer, TransportServiceServer
import json
import yaml
from ras_bt_framework.managers.BaTMan import BaTMan
from ras_interfaces.msg import BTNodeStatus
from pathlib import Path
from ras_transport.interfaces.TransportWrapper import TransportFileClient
from ras_logging.ras_logger import RasLogger
from ras_common.globals import RAS_CONFIGS_PATH, RAS_CACHE_PATH
import threading

class TrajectoryLogger(LifecycleNode):
    """
    A ROS2 node that handles receiving and executing experiment data on the robot.

    This node is responsible for:
    1. Receiving experiment data from the server
    2. Managing experiment execution on the robot
    3. Tracking executed steps for each experiment
    4. Providing cache status information
    5. Logging execution status

    The node maintains state about:
    - Currently executing experiment
    - Steps that have been executed
    - Cache status of experiments
    """

    def __init__(self):
        """
        Initialize the TrajectoryLogger node.

        Sets up:
        1. MQTT clients for communication
        2. Service servers for cache status and path loading
        3. File client for data transfer
        4. AWS IoT connection
        5. Step tracking system
        """
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
        self.remote_cache_check_server = TransportServiceServer("remote_cache_check", self.custom_callback)
        self.remote_calibration_server = TransportServiceServer("remote_calibration", self.remote_calibration_callback)
        self.batman = BaTMan()

        # Track executed steps for each experiment and hash combination
        self.executed_steps = {}  # Format: {(exp_id, hash_id): set(executed_step_numbers)}

        # Connect to AWS IoT
        self.connect_to_aws()

        self.payload = ''

    def get_executed_steps(self, exp_id, hash_id):
        """
        Gets the set of steps that have been executed for a specific experiment and hash.

        Args:
            exp_id (str): The experiment ID
            hash_id (str): The hash ID of the experiment

        Returns:
            set: Set of step numbers that have been executed
        """
        return self.executed_steps.get((exp_id, hash_id), set())

    def mark_step_executed(self, exp_id, hash_id, step_number):
        """
        Marks a step as executed for a specific experiment and hash.

        Args:
            exp_id (str): The experiment ID
            hash_id (str): The hash ID of the experiment
            step_number (int): The step number to mark as executed
        """
        if (exp_id, hash_id) not in self.executed_steps:
            self.executed_steps[(exp_id, hash_id)] = set()
        self.executed_steps[(exp_id, hash_id)].add(step_number)

    def connect_to_aws(self):
        self.remote_bt_server.connect_with_retries()
        self.remote_cache_check_server.connect_with_retries()
        self.remote_calibration_server.connect_with_retries()
        self.file_client.connect_with_retries()
        self.reciever_timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.remote_bt_server.loop()
        self.remote_cache_check_server.loop()
        self.remote_calibration_server.loop()

    def remote_calibration_callback(self, message):
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
        """
        Callback for handling incoming experiment data and cache status requests.

        This function:
        1. Handles cache status requests
        2. Extracts experiment name and hash from the message
        3. Downloads and extracts the experiment data
        4. Executes the behavior trees in sequence
        5. Tracks executed steps
        6. Logs execution status

        Args:
            message (bytes): The incoming message containing either:
                - Cache status request (JSON with experiment_id and hash_id)
                - Zip filename for experiment execution

        Returns:
            str: JSON string containing the execution status or cache status
        """
        try:
            # Try to parse as JSON first (cache status request)
            request_data = json.loads(message.decode("utf-8"))
            if isinstance(request_data, dict) and "experiment_id" in request_data and "hash_id" in request_data:
                # This is a cache status request
                experiment_id = request_data["experiment_id"]
                hash_id = request_data["hash_id"]
                self.logger.log_info(f"Received cache status request for experiment: {experiment_id} with hash: {hash_id}")

                configs_path = RAS_CACHE_PATH
                if not configs_path:
                    self.logger.log_error("Unable to find the configs path (RAS_CACHE_PATH)")
                    return json.dumps({"cache_present": False, "status": False, "message": "Config path not found"})

                # Construct the potential extraction path within the robot cache
                cache_exp_dir = os.path.join(configs_path, "robot", experiment_id, hash_id)

                # Check if the cache for this experiment and hash exists and contains trajectory files
                traj_dir_path = os.path.join(cache_exp_dir, "trajectory")
                cache_present = os.path.exists(traj_dir_path) and os.path.isdir(traj_dir_path) and any(f.endswith('.xml') or f.endswith('.txt') for f in os.listdir(traj_dir_path))

                self.logger.log_info(f"Cache status for experiment '{experiment_id}' with hash '{hash_id}': {cache_present}")

                # Call the load_path service regardless of whether the cache was just downloaded
                # or was already present. This informs TrajectoryRecordsService about the path.
                self.logger.log_info(f"Calling load_path service with path: {traj_dir_path}")
                path_req = SetPath.Request()
                path_req.path = traj_dir_path
                self.path_client.call_async(path_req)

                if cache_present:
                    self.logger.log_info(f"Cache found for experiment '{experiment_id}' with hash '{hash_id}'. Starting execution...")
                    
                    # Start timing the experiment execution
                    start_time = time.time()

                    # Find and execute all real_stepX.xml files in order
                    bt_files = sorted([f for f in os.listdir(traj_dir_path) if f.startswith("real_step") and f.endswith(".xml")],
                                    key=lambda x: int(x[len("real_step"): -len(".xml")])) # Sort by step number

                    overall_success = True
                    if not bt_files:
                        self.logger.log_warn(f"No real_stepX.xml files found in {traj_dir_path}")
                        overall_success = False
                        self.call_status_log_service('FAILED')
                    else:
                        self.logger.log_info(f"Found behavior tree steps: {bt_files}")
                        
                        # Get executed steps and execute only the new step
                        executed_steps = self.get_executed_steps(experiment_id, hash_id)
                        self.logger.log_info(f"Already executed steps for {experiment_id}: {executed_steps}")
                        
                        for bt_file in bt_files:
                            step_number = int(bt_file[len("real_step"): -len(".xml")])
                            
                            # Skip if step already executed
                            if step_number in executed_steps:
                                self.logger.log_info(f"Skipping already executed step: {bt_file}")
                                continue
                            
                            bt_path = os.path.join(traj_dir_path, bt_file)
                            step_success = self.execute_behavior_tree(bt_path, step_number, experiment_id, hash_id)
                            if not step_success:
                                overall_success = False
                                break # Stop executing further steps if one fails

                    # Calculate and log execution time
                    end_time = time.time()
                    execution_time = end_time - start_time
                    self.logger.log_info(f"Experiment execution completed in {execution_time:.2f} seconds")

                    # Return cache status and execution status
                    return json.dumps({"cache_present": cache_present, "status": overall_success, "execution_time": execution_time})

                else:
                     # If cache not present, just return the cache status
                     return json.dumps({"cache_present": cache_present})

        except json.JSONDecodeError:
            # Not a JSON message, treat as zip filename for new experiment download/execution
            pass

        # The message now contains the zip filename with hash
        self.payload = message.decode("utf-8")
        zip_filename = self.payload.strip()
        self.logger.log_info(f"Received message: {zip_filename}")

        # Start timing the experiment execution for non-cached case
        start_time = time.time()

        # Extract experiment name and hash from filename (e.g., "xml_experiment_name_hash.zip")
        try:
            if zip_filename.startswith("xml_") and zip_filename.endswith(".zip"):
                # Remove "xml_" prefix and ".zip" suffix
                name_hash = zip_filename[len("xml_"):-len(".zip")]
                # Split by last underscore to separate name and hash
                parts = name_hash.rsplit("_", 1)
                if len(parts) == 2:
                    experiment_name, hash_id = parts
                    self.logger.log_info(f"Extracted experiment name: {experiment_name}, hash: {hash_id}")
                else:
                    self.logger.log_error(f"Invalid zip filename format (missing hash): {zip_filename}")
                    status = False
                    payload = json.dumps({"status": status, "cache_found": False, "message": "Invalid filename format"})
                    return payload
            else:
                self.logger.log_error(f"Invalid zip filename format received: {zip_filename}")
                status = False
                payload = json.dumps({"status": status, "cache_found": False, "message": "Invalid filename format"})
                return payload
        except Exception as e:
            self.logger.log_error(f"Failed to extract experiment name and hash from {zip_filename}: {e}")
            status = False
            payload = json.dumps({"status": status, "cache_found": False, "message": f"Failed to extract experiment name and hash: {e}"})
            return payload

        configs_path = RAS_CACHE_PATH
        if not configs_path:
            self.logger.log_error("Unable to find the configs path (RAS_CACHE_PATH)")
            status = False
            payload = json.dumps({"status": status, "cache_found": False, "message": "Config path not found"})
            return payload

        # Construct the extraction path within the robot cache using hash
        cache_exp_dir = os.path.join(configs_path, "robot", experiment_name, hash_id)
        traj_dir_path = os.path.join(cache_exp_dir, "trajectory")

        # Check if this is a new experiment (cache not present)
        is_new_experiment = not os.path.exists(cache_exp_dir)
        if is_new_experiment:
            self.logger.log_info(f"New experiment detected: {experiment_name} with hash {hash_id}")
        else:
            self.logger.log_info(f"Existing experiment: {experiment_name} with hash {hash_id}")

        # Create directory and download new zip
        Path(traj_dir_path).mkdir(parents=True, exist_ok=True)
        local_zip_path = os.path.join(traj_dir_path, zip_filename)

        self.logger.log_info(f"Downloading the file {zip_filename} to {local_zip_path} using http...")
        result = self.file_client.download(zip_filename, local_zip_path)

        if result:
            self.logger.log_info("Download completed")
        else:
            self.logger.log_error("Download failed")
            payload = json.dumps({"status": False, "cache_found": False, "message": "Download failed"})
            return payload

        self.logger.log_info(f"Unzipping {local_zip_path} to {traj_dir_path}...")
        try:
            result2 = subprocess.run(
                ["unzip", "-o", local_zip_path, "-d", traj_dir_path],
                check=True,
                capture_output=True,
                text=True
            )
            self.logger.log_info("Unzip completed")
        except subprocess.CalledProcessError as e:
             self.logger.log_error(f"Unzip failed: {e.stderr}")
             payload = json.dumps({"status": False, "cache_found": False, "message": f"Unzip failed: {e.stderr}"})
             return payload

        # Restore the call to the load_path service
        self.logger.log_info(f"Calling load_path service with path: {traj_dir_path}")
        path_req = SetPath.Request()
        path_req.path = traj_dir_path
        self.path_client.call_async(path_req)

        # Find and execute all real_stepX.xml files in order
        bt_files = sorted([f for f in os.listdir(traj_dir_path) if f.startswith("real_step") and f.endswith(".xml")],
                          key=lambda x: int(x[len("real_step"): -len(".xml")])) # Sort by step number

        overall_success = True
        if not bt_files:
            self.logger.log_warn(f"No real_stepX.xml files found in {traj_dir_path}")
            overall_success = False
            self.call_status_log_service('FAILED')
        else:
            self.logger.log_info(f"Found behavior tree steps: {bt_files}")
            
            if is_new_experiment:
                # For new experiment, execute the first step
                bt_file = bt_files[0]
                step_number = int(bt_file[len("real_step"): -len(".xml")])
                bt_path = os.path.join(traj_dir_path, bt_file)
                self.logger.log_info(f"Executing first step of new experiment: {bt_file}")
                overall_success = self.execute_behavior_tree(bt_path, step_number, experiment_name, hash_id)
            else:
                # For existing experiment, get executed steps and execute only the new step
                executed_steps = self.get_executed_steps(experiment_name, hash_id)
                self.logger.log_info(f"Already executed steps for {experiment_name}: {executed_steps}")
                
                for bt_file in bt_files:
                    step_number = int(bt_file[len("real_step"): -len(".xml")])
                    
                    # Skip if step already executed
                    if step_number in executed_steps:
                        self.logger.log_info(f"Skipping already executed step: {bt_file}")
                        continue
                    
                    bt_path = os.path.join(traj_dir_path, bt_file)
                    step_success = self.execute_behavior_tree(bt_path, step_number, experiment_name, hash_id)
                    if not step_success:
                        overall_success = False
                        break # Stop executing further steps if one fails

        # Calculate and log execution time
        end_time = time.time()
        execution_time = end_time - start_time
        self.logger.log_info(f"Experiment execution completed in {execution_time:.2f} seconds")

        # The response payload should indicate the execution status
        payload = json.dumps({"status": overall_success, "execution_time": execution_time})
        return payload
        
    def call_status_log_service(self, status_value):
        """
        Calls the status logging service to update execution status.

        This function:
        1. Waits for the status logging service to be available
        2. Creates and sends a status update request
        3. Handles the response asynchronously

        Args:
            status_value (str): The status to log ('SUCCESS' or 'FAILED')
        """
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

    def execute_behavior_tree(self, bt_path, step_number, experiment_name, hash_id):
        """
        Executes a behavior tree and logs its status.

        This function:
        1. Executes the behavior tree
        2. Logs the execution status
        3. Marks the step as executed if successful
        4. Updates the status log service

        Args:
            bt_path (str): Path to the behavior tree XML file
            step_number (int): The step number being executed
            experiment_name (str): Name of the experiment
            hash_id (str): Hash ID of the experiment

        Returns:
            bool: True if execution was successful, False otherwise
        """
        self.logger.log_info(f"Executing behavior tree step: {bt_path}")
        status = self.batman.execute_bt(bt_path)
        
        if status not in [BTNodeStatus.SUCCESS, BTNodeStatus.IDLE]:
            self.logger.log_error(f"Behavior Tree Execution Failed with status {status}")
            self.call_status_log_service('FAILED')
            return False
        else:
            # Mark step as executed
            self.mark_step_executed(experiment_name, hash_id, step_number)
            self.logger.log_info(f"Marked step {step_number} as executed")
            self.call_status_log_service('SUCCESS')
            return True

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
        receiver.remote_cache_check_server.disconnect()
        receiver.logger.log_info("Disconnected from AWS IoT")
        receiver.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()