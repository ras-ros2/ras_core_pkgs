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

"""
ROS2 service node for recording, loading, and playing joint trajectories.

This node subscribes to a trajectory topic to save received trajectories,
provides services to load trajectory paths and play saved trajectories,
and manages a counter for saving multiple trajectories per experiment step.
"""

import os
import rclpy
from rclpy.node import Node
from pathlib import Path
from typing import List
from ras_interfaces.srv import TrajSend, SetPath, PlayPath, ActionTraj
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.serialization import serialize_message
import json
import hashlib
from rosidl_runtime_py.set_message import set_message_fields
from builtin_interfaces.msg import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_common.globals import RAS_CONFIGS_PATH, RAS_CACHE_PATH

class TrajectoryRecordsService(Node):
    """
    A ROS2 node for managing and playing robot joint trajectories.
    
    This service records trajectories received on a topic and provides
    functionality to load a directory containing trajectory files and
    play specific trajectories by their UUID.
    
    Attributes:
        my_callback_group (ReentrantCallbackGroup): Callback group for handling service requests
        trajectory_path (str): The current directory path containing trajectory files.
        current_experiment_id (str): The ID of the currently loaded experiment.
        current_step (int): The current step number within the experiment.
        counter (int): A counter used for naming saved trajectory files sequentially within a step.
    """
    def __init__(self):
        """
        Initialize the TrajectoryRecordsService node.
        
        Sets up subscriptions, services, and initializes internal state variables.
        """
        super().__init__("trajectory_recoder_service")
        self.get_logger().info("Node Initialized")
        self.my_callback_group = ReentrantCallbackGroup()
        self.trajectory_path = None
        self.current_experiment_id = None
        self.current_step = 1  # Track current step
           
        self.create_subscription(JointTrajectory, "trajectory_topic", self.save_trajectory, 10)
        self.create_service(SetPath, '/load_path', self.load_path, callback_group=self.my_callback_group)
        self.create_service(PlayPath, '/play_trajectory', self.play_trajectory, callback_group=self.my_callback_group)
        self.create_service(SetBool, '/reset_counter', self.counter_reset_callback, callback_group=self.my_callback_group)
        self.counter = 0

    def _read_experiment_id(self) -> str:
        """
        Read experiment ID from current_experiment.txt.
        
        This file is expected to be written by the ExperimentService.
        
        Returns:
            str: The experiment ID, or None if the file cannot be read or is empty.
        """
        try:
            with open(os.path.join(RAS_CONFIGS_PATH, "current_experiment.txt"), 'r') as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"Failed to read experiment ID from file: {e}")
            return None

    def _read_experiment_hash(self, exp_id: str) -> str:
        """
        Read experiment hash from experiment_file_hashes.json.
        
        This file is expected to contain a mapping of experiment IDs to their hashes.
        
        Args:
            exp_id (str): The ID of the experiment.
            
        Returns:
            str: The hash ID for the experiment, or "no_hash" if not found or an error occurs.
        """
        try:
            hash_file = os.path.join(RAS_CONFIGS_PATH, "experiment_file_hashes.json")
            if os.path.exists(hash_file) and os.path.getsize(hash_file) > 0:
                with open(hash_file, 'r') as f:
                    hashes = json.load(f)
                    if exp_id in hashes:
                        return hashes[exp_id]
            self.get_logger().warn(f"No hash found for experiment {exp_id}")
            return "no_hash"
        except Exception as e:
            self.get_logger().error(f"Failed to read experiment hash from file: {e}")
            return "no_hash"

    def experiment_callback(self, msg):
        """
        Callback to handle experiment ID updates.
        
        Note: This callback is currently unused based on the provided code snippet.
        
        Args:
            msg: The incoming message containing the experiment ID.
        """
        self.get_logger().info(f"Current experiment ID: {self.current_experiment_id}")
    
    def counter_reset_callback(self, req, resp):
        """
        Reset the trajectory counter and increment the step number.
        
        This is typically called after a step's trajectories have been recorded.
        
        Args:
            req (SetBool.Request): The service request.
            resp (SetBool.Response): The service response.
            
        Returns:
            SetBool.Response: The response with success status.
        """
        self.counter = 0
        self.current_step += 1  # Increment step number when counter is reset
        self.get_logger().info(f"Counter reset. Moving to step {self.current_step}")
        resp.success = True
        return resp
        
    def convert_json_to_msg(self, json_obj:dict|str, msg_type: type):
        """
        Converts a JSON object or string to a ROS2 message of a specified type.
        
        Args:
            json_obj (dict | str): The JSON data as a dictionary or string.
            msg_type (type): The ROS2 message type to convert to.
            
        Returns:
            msg_type: The populated ROS2 message object.
            
        Raises:
            AssertionError: If a field in the JSON object is not found in the message type.
        """
        assert isinstance(msg_type, type)
        if isinstance(json_obj,str):
            json_obj = json.loads(json_obj)
        msg = msg_type()
        for field in json_obj.keys():
            assert hasattr(msg, field), f"Attribute '{field}' not found in message type {msg_type.__name__}"
        set_message_fields(msg,json_obj)
        return msg

    def save_trajectory(self, msg):
        """
        Callback function for the "trajectory_topic" subscription.
        
        Saves the received JointTrajectory message to a file in both the package's
        trajectory directory and the experiment's cache directory.
        
        Args:
            msg (JointTrajectory): The received joint trajectory message.
        """
        trajectory_data = {
                "joint_names": msg.joint_names,
                "points": [{
                    "positions": list(point.positions),
                    "velocities": list(point.velocities) if point.velocities else [],
                    "accelerations": list(point.accelerations) if point.accelerations else [],
                    "effort": list(point.effort) if point.effort else [],
                    "time_from_start": {
                        "sec": point.time_from_start.sec,
                        "nanosec": point.time_from_start.nanosec
                    }
                } for point in msg.points]
            }
        self.current_experiment_id = self._read_experiment_id()
        if not self.current_experiment_id:
            self.get_logger().error("No experiment ID found, cannot save trajectory")
            return

        # Get the hash for current experiment
        hash_id = self._read_experiment_hash(self.current_experiment_id)
        self.get_logger().info(f"Using hash {hash_id} for experiment {self.current_experiment_id}")
        
        self.counter += 1  # Increment the counter
        
        # Use counter as the file number (1.txt, 2.txt, 3.txt)
        file_number = str(self.counter)
        self.get_logger().info(f"Generating trajectory file: {file_number}.txt")
        
        pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
        configs_path = os.path.join(RAS_CACHE_PATH)
        if pkg_path is None:
            raise RuntimeError(f"Invalid package path")
            
        # Save to package trajectory directory
        trajectory_dir = os.path.join(str(pkg_path), "xml", "trajectory")
        # os.makedirs(trajectory_dir, exist_ok=True)  # Ensure directory exists
        with open(os.path.join(trajectory_dir, f"{file_number}.txt"), 'w') as file:
            file.write(f"{trajectory_data}")
        self.get_logger().info(f"Saved trajectory to package directory: {file_number}.txt")
        
        # Save to experiment cache directory with hash
        cache_dir = os.path.join(str(configs_path), "server", self.current_experiment_id, hash_id, "trajectory")
        # os.makedirs(cache_dir, exist_ok=True)  # Ensure directory exists
        with open(os.path.join(cache_dir, f"{file_number}.txt"), 'w') as file:
            file.write(f"{trajectory_data}")
        self.get_logger().info(f"Saved trajectory to cache directory: {file_number}.txt")

    def load_trajectory(self, uuid: str) -> JointTrajectory:
        """
        Loads a specific trajectory from the current trajectory directory.
        
        Args:
            uuid (str): The unique ID (filename without extension) of the trajectory to load.
            
        Returns:
            JointTrajectory: The loaded joint trajectory message.
            
        Raises:
            FileNotFoundError: If the trajectory file does not exist.
            json.JSONDecodeError: If the file content is not valid JSON.
            KeyError: If expected keys are missing in the JSON data.
        """
        path = self.trajectory_path
        if not path:
            self.get_logger().error("Trajectory path not set. Call /load_path first.")
            raise FileNotFoundError("Trajectory path not set")
            
        final_path = path +f"/{uuid}.txt"
        with open(final_path, 'r') as file:
            for line in file:
                traj_string = line.strip()
        
        json_string = traj_string.replace("'", '"')

        trajectory_data = json.loads(json_string)
        print(type(trajectory_data))
        # print(trajectory_data["size_of_json"])
        joint_trajectory = JointTrajectory()
        print(trajectory_data['joint_names'])
        joint_trajectory.joint_names = trajectory_data['joint_names']
        joint_trajectory.points = [
            JointTrajectoryPoint(
                positions=point['positions'],
                velocities=point.get('velocities', []),
                accelerations=point.get('accelerations', []),
                effort=point.get('effort', []),
                time_from_start=Duration(sec=point['time_from_start']['sec'], nanosec=point['time_from_start']['nanosec'])
            ) for point in trajectory_data['points']
        ]
        return joint_trajectory
        
    def play_trajectory(self, req, resp):
        """
        Callback function for the /play_trajectory service.
        
        Loads a trajectory by UUID and sends it as a goal to an ActionTraj action server.
        
        Args:
            req (PlayPath.Request): The service request containing the unique_id and topic_name.
            resp (PlayPath.Response): The service response.
            
        Returns:
            PlayPath.Response: The response with success status based on the action result.
        """
        uuid = req.unique_id
        try:
            traj = self.load_trajectory(uuid)
        except FileNotFoundError:
            self.get_logger().error(f"Trajectory file not found for UUID: {uuid}")
            resp.success = False
            return resp
        except Exception as e:
            self.get_logger().error(f"Error loading trajectory with UUID {uuid}: {e}")
            resp.success = False
            return resp
            
        topic_name = req.topic_name
        traj_client = self.create_client(ActionTraj, topic_name, callback_group=self.my_callback_group)
        
        # Wait for action server to be available
        if not traj_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Action server {topic_name} not available.")
            resp.success = False
            return resp
            
        request = ActionTraj.Request()
        request.traj = traj
        future = traj_client.call_async(request)
        print(f"Playing trajectory with UUID: {uuid} on topic {topic_name}")
        
        # Spin until the future is complete (action result is received)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            # Assuming the action result has a 'success' field
            if hasattr(future.result(), 'success'):
                self.get_logger().info(f"Action result success: {future.result().success}")
                resp.success = future.result().success
            else:
                self.get_logger().warn("Action result does not have a 'success' field.")
                resp.success = False
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            resp.success = False
            
        return resp

    def load_path(self, req, resp):
        """
        Callback function for the /load_path service.
        
        Sets the directory path where trajectory files are located.
        
        Args:
            req (SetPath.Request): The service request containing the path.
            resp (SetPath.Response): The service response.
            
        Returns:
            SetPath.Response: The response indicating success.
        """
        self.trajectory_path = req.path
        self.get_logger().info(f"Loaded trajectory path: {self.trajectory_path}")
        return resp

def main():
    rclpy.init(args=None)
    node = TrajectoryRecordsService()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
