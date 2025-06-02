import os
import yaml
import datetime
import json
import time
import signal
import threading
import subprocess
import psutil
import hashlib
import shutil
from ras_bt_framework.behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behavior_utility.update_bt import update_xml, update_bt
import xml.etree.ElementTree as ET
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ras_interfaces.srv import LoadExp
from std_srvs.srv import SetBool
from .BaTMan import BaTMan
from pathlib import Path
from ras_interfaces.msg import BTNodeStatus
from ..generators.behavior_tree_generator import BehaviorTreeGenerator
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_common.globals import RAS_CONFIGS_PATH, RAS_CACHE_PATH
import time
from ..behavior_template.module import BehaviorModuleSequence
import threading
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
from ras_interfaces.action import ExecuteExp
import subprocess
from ras_logging.ras_logger import RasLogger
from ras_interfaces.srv import ReportRobotState
from std_srvs.srv import Trigger
from ras_bt_framework.config import action_mapping

# Define a new service type for checking cache status
# Assuming a srv file named CheckCacheStatus.srv exists in ras_interfaces/srv
# Request: string experiment_id
# Response: bool cache_present
from ras_interfaces.srv import CheckCacheStatus

class ExperimentService(Node):
    """
    A ROS2 service node for managing and executing robot experiments.
    
    This service provides functionality to load experiments from YAML files,
    execute behavior trees, and manage experiment-related services.
    
    Attributes:
        my_callback_group (ReentrantCallbackGroup): Callback group for handling service requests
        counter_reset_client (Client): Client for resetting the experiment counter
        batman (BaTMan): Instance of BaTMan for managing experiment sequences
    """
    # File to persist experiment state
    STATE_FILE = os.path.join(RAS_CONFIGS_PATH, "experiment_state.json")

    # File to store experiment hashes
    HASH_FILE = os.path.join(RAS_CONFIGS_PATH, "experiment_file_hashes.json")

    # # File to store experiment trajectory folder hashes
    # TRAJ_HASH_FOLDER = os.path.join(RAS_CONFIGS_PATH, "experiment_traj_hashes.json")

    # Directory for caching behavior trees and trajectories
    CACHE_DIR = os.path.join(RAS_CACHE_PATH, "server")

    def __init__(self):
        """
        Initialize the ExperimentService node.
        
        This sets up the necessary services and clients for experiment management.
        """
        # Create cache directory if it doesn't exist
        os.makedirs(self.CACHE_DIR, exist_ok=True)
        super().__init__("experiment_service")
        self.logger = RasLogger()
        self.my_callback_group = ReentrantCallbackGroup()
        self.create_service(LoadExp, "/execute_experiment", self.execute_experiment_callback, callback_group=self.my_callback_group)
        self.create_service(SetBool, "/sim_step", self.sim_step_callback, callback_group=self.my_callback_group)
        self.create_service(SetBool, "/next_step", self.next_step_callback, callback_group=self.my_callback_group)
        self.counter_reset_client = self.create_client(SetBool, '/reset_counter', callback_group=self.my_callback_group)
        
        # Add action client for execute_exp
        self.execute_exp_client = ActionClient(self, ExecuteExp, '/execute_exp', callback_group=self.my_callback_group)
        
        self.batman = BaTMan()
        
        # Track current step in execution
        self.pose_dict = None
        self.target_sequence = None
        self.current_step_index = 0
        self.total_steps = 0
        self.sim_complete = False
        self.experiment_changed = False
        
        # Threading lock for experiment execution
        self.exp_execution_active = False
        self.exp_execution_thread = None
        self.exp_execution_lock = threading.Lock()
        self.wait_for_real_robot = threading.Event()
        self.current_experiment_id = None

        self.robot_state =  {"state": "unknown", "details": ""}

        self.create_service(ReportRobotState, "/report_robot_state", self.report_robot_state_callback, callback_group=self.my_callback_group)
        
        # Create cache status service client
        # Assuming CheckCacheStatus.srv exists in ras_interfaces/srv
        self.cache_status_client = self.create_client(CheckCacheStatus, 'cache_status', callback_group=self.my_callback_group)

        # Attempt to recover experiment state on startup
        self.recover_experiment_state()

        # Add /resume_experiment service
        self.create_service(Trigger, "/resume_experiment", self.resume_experiment_callback)

    # def load_traj_hash(self, exp_id):
    #     if os.path.exists(self.TRAJ_HASH_FOLDER):
    #         if os.path.getsize(self.TRAJ_HASH_FOLDER) == 0:
    #             return None
    #         try:
    #             with open(self.TRAJ_HASH_FOLDER, 'r') as f:
    #                 hashes = json.load(f)
    #             return hashes.get(exp_id)
    #         except Exception as e:
    #             self.logger.log_error(f"Error loading traj hashes: {e}")
    #     return None
    
    # def calculate_traj_hash(self, traj_hash_path):
    #     """Compute SHA256 hash over all files in folder (sorted order, including file content and relative path)."""
    #     hash_sha256 = hashlib.sha256()
    #     for root, dirs, files in os.walk(folder_path):
    #         files.sort()
    #         for fname in files:
    #             fpath = os.path.join(root, fname)
    #             rel_path = os.path.relpath(fpath, folder_path)
    #             hash_sha256.update(rel_path.encode())
    #             with open(fpath, 'rb') as f:
    #                 while True:
    #                     chunk = f.read(4096)
    #                     if not chunk:
    #                         break
    #                     hash_sha256.update(chunk)
    #     return hash_sha256.hexdigest()

    # def save_traj_hash(self, exp_id, traj_hash):
    #     try:
    #         hashes = {}
    #         if os.path.exists(self.TRAJ_HASH_FOLDER) and os.path.getsize(self.TRAJ_HASH_FOLDER) > 0:
    #             with open(self.TRAJ_HASH_FOLDER, 'r') as f:
    #                 hashes = json.load(f)
    #         hashes[exp_id] = traj_hash
    #         with open(self.TRAJ_HASH_FOLDER, 'w') as f:
    #             json.dump(hashes, f, indent=2)
    #         self.logger.log_info(f"Saved folder hash for experiment {exp_id}")
    #     except Exception as e:
    #         self.logger.log_error(f"Error saving folder hash: {e}")

    def save_experiment_state(self):
        state = {
            "experiment_id": getattr(self, "current_experiment_id", None),
            "current_step_index": getattr(self, "current_step_index", None),
            "timestamp": datetime.datetime.now().isoformat()
        }
        try:
            with open(self.STATE_FILE, 'w') as f:
                json.dump(state, f)
            self.logger.log_info(f"Experiment state saved: {state}")
        except Exception as e:
            self.logger.log_error(f"Failed to save experiment state: {e}")
            
    def get_experiment_cache_dir(self, exp_id):
        """Get the cache directory for an experiment."""
        # Get the current hash for this experiment
        hashes = self.load_experiment_hashes()
        current_hash = hashes.get(exp_id)
        
        if current_hash is None:
            # If no hash exists, calculate it from the experiment file
            exp_path = os.path.join(RAS_CONFIGS_PATH, "experiments", f"{exp_id}.yaml")
            current_hash = self.calculate_file_hash(exp_path)
            if current_hash:
                # Save the newly calculated hash
                self.save_experiment_hash(exp_id, current_hash)
                self.logger.log_info(f"Calculated and saved new hash for experiment {exp_id}: {current_hash}")
            else:
                self.logger.log_error(f"Could not calculate hash for experiment {exp_id}")
                return None
        
        # Use the hash in the path
        exp_cache_dir = os.path.join(self.CACHE_DIR, exp_id, current_hash, "trajectory")
        os.makedirs(exp_cache_dir, exist_ok=True)
        return exp_cache_dir
        
    def get_trajectory_cache_dir(self, exp_id):
        """Get the trajectory cache directory for an experiment."""
        traj_cache_dir = self.get_experiment_cache_dir(exp_id)
        return traj_cache_dir
        
    def cache_behavior_tree(self, exp_id, step_number, source_path):
        """Cache a behavior tree for a specific experiment step."""
        exp_cache_dir = self.get_experiment_cache_dir(exp_id)
        target_file = os.path.join(exp_cache_dir, f"real_step{step_number}.xml")
        
        try:
            shutil.copy(source_path, target_file)
            self.logger.log_info(f"Cached behavior tree for experiment {exp_id}, step {step_number}")
            return target_file
        except Exception as e:
            self.logger.log_error(f"Error caching behavior tree: {e}")
            return None
            
    def _files_are_identical(self, file1, file2):
        """Check if two files are identical by comparing their contents."""
        try:
            with open(file1, 'rb') as f1, open(file2, 'rb') as f2:
                return f1.read() == f2.read()
        except Exception:
            return False
            
    def cache_trajectories(self, exp_id, source_dir):
        """Cache all trajectory files from a source directory."""
        traj_cache_dir = self.get_trajectory_cache_dir(exp_id)
        
        try:
            # Create trajectory directory if it doesn't exist
            os.makedirs(traj_cache_dir, exist_ok=True)
            return True
        except Exception as e:
            self.logger.log_error(f"Error caching trajectories: {e}")
            return False
            
    def get_cached_behavior_tree(self, exp_id, step_number):
        """Get a cached behavior tree for a specific experiment step."""
        exp_cache_dir = self.get_experiment_cache_dir(exp_id)
        target_file = os.path.join(exp_cache_dir, f"real_step{step_number}.xml")
        
        if os.path.exists(target_file):
            self.logger.log_info(f"Found cached behavior tree for experiment {exp_id}, step {step_number}")
            return target_file
        else:
            self.logger.log_info(f"No cached behavior tree found for experiment {exp_id}, step {step_number}")
            return None
            
    def experiment_is_cached(self, exp_id):
        """Check if an experiment has any cached files."""
        exp_cache_dir = self.get_experiment_cache_dir(exp_id)
        
        # Check if the experiment directory exists and has any files
        if os.path.exists(exp_cache_dir) and os.listdir(exp_cache_dir):
            return True
        else:
            return False
            
    def clear_experiment_cache(self, exp_id):
        """Clear the cache for a specific experiment."""
        exp_cache_dir = self.get_experiment_cache_dir(exp_id)
        
        try:
            if os.path.exists(exp_cache_dir):
                shutil.rmtree(exp_cache_dir)
                os.makedirs(exp_cache_dir, exist_ok=True)
                self.logger.log_info(f"Cleared cache for experiment {exp_id}")
                return True
            else:
                self.logger.log_info(f"No cache found for experiment {exp_id}")
                return True
        except Exception as e:
            self.logger.log_error(f"Error clearing experiment cache: {e}")
            return False

    def calculate_file_hash(self, file_path):
        """
        Calculate the SHA-256 hash of a file.
        
        Args:
            file_path (str): Path to the file to hash
            
        Returns:
            str: Hexadecimal digest of the file hash
        """
        if not os.path.exists(file_path):
            self.logger.log_error(f"File not found for hashing: {file_path}")
            return None
            
        try:
            with open(file_path, 'rb') as f:
                file_hash = hashlib.sha256(f.read()).hexdigest()
                self.logger.log_info(f"Calculated hash for {os.path.basename(file_path)}: {file_hash[:8]}...")
                return file_hash
        except Exception as e:
            self.logger.log_error(f"Error calculating file hash: {e}")
            return None
    
    def load_experiment_hashes(self):
        """
        Load experiment hashes from file if it exists.
        
        Returns:
            dict: Dictionary of experiment IDs and their hashes
        """
        if os.path.exists(self.HASH_FILE):
            try:
                with open(self.HASH_FILE, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.logger.log_error(f"Error loading experiment hashes: {e}")
        return {}
        
    def save_experiment_hash(self, exp_id, file_hash):
        """
        Save an experiment hash to the hash file for record keeping.
        
        Args:
            exp_id (str): Experiment ID
            file_hash (str): Hash of the experiment file
        """
        try:
            hashes = {}
            if os.path.exists(self.HASH_FILE) and os.path.getsize(self.HASH_FILE) > 0:
                with open(self.HASH_FILE, 'r') as f:
                    hashes = json.load(f)
            hashes[exp_id] = file_hash
            with open(self.HASH_FILE, 'w') as f:
                json.dump(hashes, f, indent=2)
            self.logger.log_info(f"Saved hash for experiment {exp_id} for record keeping")
        except Exception as e:
            self.logger.log_error(f"Error saving experiment hash: {e}")
    
    def check_experiment_changed(self, exp_id, file_path):
        """
        Check if an experiment file has changed by comparing its hash with the cache directory structure.
        
        Args:
            exp_id (str): Experiment ID
            file_path (str): Path to the experiment file
            
        Returns:
            tuple: (bool, str) - (has_changed, current_hash)
                - has_changed: True if the file has changed, False if unchanged or first time loading
                - current_hash: The current hash of the file
                - is_first_time: True if this is the first time loading this experiment
        """
        # Calculate current hash
        current_hash = self.calculate_file_hash(file_path)
        
        if not current_hash:
            return True, None, False
            
        # Check if experiment directory exists
        exp_dir = os.path.join(self.CACHE_DIR, exp_id)
        if not os.path.exists(exp_dir):
            self.logger.log_info(f"No previous cache found for experiment {exp_id} - first time loading")
            # Save hash for record keeping
            self.save_experiment_hash(exp_id, current_hash)
            return False, current_hash, True  # Not changed (first time), with hash and is_first_time=True
            
        # Get all subdirectories (each representing a hash)
        try:
            hash_dirs = [d for d in os.listdir(exp_dir) if os.path.isdir(os.path.join(exp_dir, d))]
        except Exception as e:
            self.logger.log_error(f"Error reading cache directory: {e}")
            return True, current_hash, False
            
        # If no hash directories exist, this is first time
        if not hash_dirs:
            self.logger.log_info(f"No previous hash found for experiment {exp_id} - first time loading")
            # Save hash for record keeping
            self.save_experiment_hash(exp_id, current_hash)
            return False, current_hash, True
            
        # Check if current hash matches any existing hash directory
        has_changed = current_hash not in hash_dirs
        
        if has_changed:
            self.logger.log_info(f"Experiment {exp_id} has changed (previous hash not found in cache)")
            # Save new hash for record keeping
            self.save_experiment_hash(exp_id, current_hash)
        else:
            self.logger.log_info(f"Experiment {exp_id} is unchanged (hash found in cache: {current_hash})")
            
        return has_changed, current_hash, False  # has_changed value, hash, and is_first_time=False
    
    def load_experiment_state(self):
        if not os.path.exists(self.STATE_FILE):
            return None
        try:
            with open(self.STATE_FILE, 'r') as f:
                state = json.load(f)
            self.logger.log_info(f"Experiment state loaded: {state}")
            return state
        except Exception as e:
            self.logger.log_error(f"Failed to load experiment state: {e}")
            return None

    def recover_experiment_state(self):
        state = self.load_experiment_state()
        if state is not None:
            exp_id = state.get("experiment_id")
            step_idx = state.get("current_step_index", 0)
            if exp_id is not None:
                path = os.path.join(RAS_CONFIGS_PATH, "experiments", f"{exp_id}.yaml")
                if Path(path).exists():
                    self.pose_dict, self.target_sequence = read_yaml_to_pose_dict(path)
                    # Register all poses
                    if self.pose_dict is not None:
                        for pose_name, pose in self.pose_dict.items():
                            action_mapping.register_pose(pose_name, pose)
                    # self.batman.generate_module_from_keywords(self.target_sequence, self.pose_dict)
                    self.current_step_index = step_idx
                    self.total_steps = len(self.target_sequence)
                    self.sim_complete = False
                    self.current_experiment_id = exp_id
                    self.logger.log_info(f"Recovered experiment '{exp_id}' at step {step_idx}")
                else:
                    self.logger.log_error(f"Experiment file for recovery not found: {path}")
            else:
                self.logger.log_info("No experiment ID found in saved state.")
        else:
            self.logger.log_info("No experiment state to recover.")
        self.decide_recovery_action()

    def report_robot_state_callback(self, request, response):
        """
        Callback for /report_robot_state service. Stores the latest robot state.
        """

        self.robot_state = {"state": request.state, "details": request.details}
        self.logger.log_info(f"Robot state updated: {self.robot_state}")
        self.decide_recovery_action()
        response.success = True
        response.message = f"Received robot state: {request.state}"
        return response

    def decide_recovery_action(self):
        """
        Decide the appropriate recovery action based on experiment and robot state.
        Automatically resume experiment if safe.
        """

        exp_state = getattr(self, 'current_experiment_id', None)
        step_idx = getattr(self, 'current_step_index', None)
        robot_state = self.robot_state.get('state', 'unknown') if hasattr(self, 'robot_state') else 'unknown'
        if not exp_state or step_idx is None:
            self.logger.log_info("No persisted experiment state. No recovery needed.")
            return

        if robot_state in ["idle", "paused"]:
            self.logger.log_info(f"Safe to resume experiment '{exp_state}' at step {step_idx}. Automatically resuming experiment.")
            # Auto-resume logic
            with self.exp_execution_lock:
                if not self.exp_execution_active:
                    self.exp_execution_active = True
                    self.exp_execution_thread = threading.Thread(target=self.exp_execution_thread_func)
                    self.exp_execution_thread.daemon = True
                    self.exp_execution_thread.start()
                else:
                    self.logger.log_info("Experiment execution already running.")
        elif robot_state == "running":
            self.logger.log_warn(f"Robot is running but experiment state is persisted. Manual intervention required before resuming experiment '{exp_state}' at step {step_idx}.")
        elif robot_state == "error":
            self.logger.log_error(f"Robot is in error state. Manual intervention required before any experiment recovery.")
        else:
            self.logger.log_warn(f"Robot state unknown. Experiment is paused. Run /resume_experiment to continue experiment '{exp_state}' at step {step_idx}.")
        
        self.logger.log_info(f"Ensure the object width should be within the range of 2.2 to 3.8 centimeters as per the gripper constraints.")

    def resume_experiment_callback(self, req, resp):
        with self.exp_execution_lock:
            if not self.exp_execution_active:
                self.exp_execution_active = True
                self.exp_execution_thread = threading.Thread(target=self.exp_execution_thread_func)
                self.exp_execution_thread.daemon = True
                self.exp_execution_thread.start()
                resp.success = True
                resp.message = "Experiment resumed."
                self.logger.log_info("Experiment resumed via /resume_experiment service.")
            else:
                resp.success = False
                resp.message = "Experiment already running."
                self.logger.log_info("Resume requested but experiment already running.")
        return resp

    def simulate_current_step(self):
        """
        Simulate the current step in the sequence:
        1. Generate simulation trajectory
        2. Run simulation
        3. Generate real robot trajectory (but don't execute it)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.current_step_index >= self.total_steps:
            self.logger.log_info("All steps completed successfully")
            return True
            
        # Get package path for XML files
        pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
        if pkg_path is None:
            self.logger.log_error("ras_bt_framework package path not found")
            return False
            
        sim_path = Path(pkg_path) / "xml" / "sim.xml"
        real_path = Path(pkg_path) / "xml" / "real.xml"
        
        # Check if we can use cached behavior tree
        cached_bt_path = self.get_cached_behavior_tree(self.current_experiment_id, self.current_step_index + 1)
        use_cached = False
        
        if cached_bt_path and not self.experiment_changed:
            self.logger.log_info(f"Using cached behavior tree for step {self.current_step_index + 1}")
            # Copy cached behavior tree to real.xml
            try:
                shutil.copy(cached_bt_path, real_path)
                use_cached = True
            except Exception as e:
                self.logger.log_error(f"Error copying cached behavior tree: {e}")
                use_cached = False
        
        # Get current step
        current_step = [self.target_sequence[self.current_step_index]]
        step_name = f"Step{self.current_step_index + 1}"
        
        self.logger.log_info(f"Processing step {self.current_step_index + 1}/{self.total_steps}: {current_step}")
        
        # Reset counter for this step
        # counter_reset = SetBool.Request()
        # counter_reset.data = True
        # self.counter_reset_client.call_async(counter_reset)
        
        if not use_cached:
            # Generate module for single step
            step_module = BehaviorModuleSequence()
            for step in current_step:
                # If step is a dict like {'PlaceObject': 'above1'}
                if isinstance(step, dict):
                    for keyword, params in step.items():
                        if keyword in action_mapping._mappings:
                            # If params is a dict, pass as kwargs; else as a single argument
                            if isinstance(params, dict):
                                action = action_mapping.get_action(keyword)(**params)
                            else:
                                action = action_mapping.get_action(keyword)(params)
                            step_module.add_child(action)
                        else:
                            self.logger.log_warn(f"Unknown keyword: {keyword}")
                # If step is just a string keyword
                elif isinstance(step, str):
                    if step in action_mapping._mappings:
                        action = action_mapping.get_action(step)()
                        step_module.add_child(action)
                    else:
                        self.logger.log_warn(f"Unknown keyword: {step}")
            
            # Generate XML for simulation
            self.logger.log_info(f"Generating simulation trajectory for step {self.current_step_index + 1}...")
            btg = BehaviorTreeGenerator(self.batman.alfred)
            btg.feed_root(step_module)
            
            try:
                btg.generate_xml_trees(str(sim_path))
            except Exception as e:
                self.logger.log_error(f"Error generating trajectory for simulation: {e}", e)
                return False
                
            # Run simulation
            self.logger.log_info(f"Running simulation for step {self.current_step_index + 1}...")
            sim_status = self.batman.execute_bt(sim_path)
            
            if sim_status not in [BTNodeStatus.SUCCESS, BTNodeStatus.IDLE]:
                self.logger.log_error(f"Simulation failed for step {self.current_step_index + 1}")
                return False
                
            self.logger.log_info(f"Simulation successful for step {self.current_step_index + 1}")
            
            # Generate real robot XML, but don't execute it yet
            self.logger.log_info(f"Generating real robot trajectory for step {self.current_step_index + 1}...")
            real_module = update_bt(step_module)
            btg = BehaviorTreeGenerator(self.batman.alfred)
            btg.feed_root(real_module)
            
            try:
                btg.generate_xml_trees(str(real_path))
                self.logger.log_info(f"Real robot trajectory generated at {real_path}")

                # Cache the generated behavior tree if this is the first time or experiment has changed
                if self.experiment_changed or not self.experiment_is_cached(self.current_experiment_id):
                    self.cache_behavior_tree(self.current_experiment_id, self.current_step_index + 1, str(real_path))
                    
                    # Cache trajectory files
                    traj_dir = os.path.join(pkg_path, "xml", "trajectory")
                    # if os.path.exists(traj_dir):
                    #     self.cache_trajectories(self.current_experiment_id, traj_dir)
                        
                self.logger.log_info("Sending execution command to real robot...")
                
                # Execute experiment on real robot using ROS2 action
                self.send_execute_exp_goal()
                
            except Exception as e:
                self.logger.log_error(f"Error generating trajectory for real robot: {e}", e)
                return False
        
        # else:
        #     # We're using cached behavior tree, check if we need to copy trajectory files
        #     # traj_cache_dir = self.get_trajectory_cache_dir(self.current_experiment_id)
        #     traj_dir = os.path.join(pkg_path, "xml", "trajectory")
            
        #     # Ensure trajectory directory exists
        #     os.makedirs(traj_dir, exist_ok=True)

        self.sim_complete = True
        return True
    
    def send_execute_exp_goal(self):
        """
        Send a goal to the /execute_exp action server to run the experiment on the real robot.
        Includes the current experiment hash in the goal message.
        """
        # Wait for action server
        if not self.execute_exp_client.wait_for_server(timeout_sec=1.0):
            self.logger.log_info("Action server not available, using command line approach instead")
            # Fall back to using command line
            try:
                # Include hash in the command line message
                hash_msg = f"{self.current_experiment_hash}" if hasattr(self, 'current_experiment_hash') else "no_hash"
                cmd = f"ros2 action send_goal /execute_exp ras_interfaces/action/ExecuteExp '{{hash_id: \"{hash_msg}\"}}'"
                self.logger.log_info(f"Executing command: {cmd}")
                # Execute the command in a non-blocking way
                subprocess.Popen(cmd, shell=True)
                self.logger.log_info("Command sent to execute experiment on real robot")
            except Exception as e:
                self.logger.log_error(f"Failed to execute command: {e}", e)
            return
        
        # Create and send goal
        goal_msg = ExecuteExp.Goal()
        # Add hash to the goal message
        if hasattr(self, 'current_experiment_hash'):
            goal_msg.hash_id = self.current_experiment_hash
        else:
            goal_msg.hash_id = "no_hash"
        
        # Send the goal asynchronously
        send_goal_future = self.execute_exp_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.logger.log_info(f"Goal sent to execute experiment on real robot with hash: {goal_msg.hash_id}")
    
    def goal_response_callback(self, future):
        """
        Callback for handling the goal response.
        
        Args:
            future: Future object containing the goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.log_info("Goal rejected")
            return
            
        # self.logger.log_info("Goal accepted")
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Callback for handling the action result.
        
        Args:
            future: Future object containing the action result
        """
        try:
            result = future.result().result
            self.logger.log_info(f"Real robot execution completed with result: {result}")
            
            # If experiment execution is active, check result status
            if self.exp_execution_active and self.sim_complete:
                if result.success:
                    # Success case - proceed as normal
                    self.wait_for_real_robot.set()
                else:
                    # Failure case - log error and signal to continue
                    self.logger.log_error("Real robot execution failed.")
                    self.logger.log_info("Signaling to continue to next step despite failure.")
                    self.wait_for_real_robot.set()
                
        except Exception as e:
            self.logger.log_error(f"Error in get_result_callback: {e}", e)
    
    def restart_transport_services(self):
        """
        Restarts the transport services (log_receiver, tot_sender, transport_server_service)
        and relaunches the transport server after a 60-second wait time.
        """
        try:
            self.logger.log_info("Initiating transport services restart...")
            
            # Find and kill the processes by name
            processes_to_kill = ["log_receiver.py", "iot_sender.py", "transport_server_service.py"]
            
            # First kill the terminal session by sending CTRL+C to tmux pane
            self.logger.log_info("Sending CTRL+C to kill transport server terminal session...")
            try:
                subprocess.run("tmux send-keys -t main_session:1.2 C-c", shell=True, check=True)
                time.sleep(2)  # Give CTRL+C some time to take effect
            except Exception as e:
                self.logger.log_error(f"Error sending CTRL+C to tmux: {e}")
            
            # Then kill any remaining processes
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if not cmdline:
                        continue
                    
                    proc_cmd = ' '.join(cmdline)
                    
                    if any(proc_name in proc_cmd for proc_name in processes_to_kill):
                        self.logger.log_info(f"Killing process: {proc.info['pid']} - {proc_cmd}")
                        try:
                            os.kill(proc.info['pid'], signal.SIGTERM)
                        except Exception as e:
                            self.logger.log_error(f"Error killing process {proc.info['pid']}: {e}")
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass
            
            # Wait for 60 seconds
            self.logger.log_info("Waiting 10 seconds before restarting transport services...")
            time.sleep(10)
            
            # Relaunch the transport server using tmux
            self.logger.log_info("Relaunching transport server...")
            cmd = "tmux send-keys -t main_session:1.2 \"ros2 launch ras_transport server.launch.py\" C-m"
            subprocess.run(cmd, shell=True, check=True)
            self.logger.log_info("Transport server relaunched successfully")
            self.logger.log_info("1. You can now resume the experiment using resume_experiment service")
            self.logger.log_info("OR")
            self.logger.log_info("2. You can start a new experiment using execute_experiment service")
            
        except Exception as e:
            self.logger.log_error(f"Error in restart_transport_services: {e}")
    
    def sim_step_callback(self, req, resp):
        """
        Simulate the current step of the loaded experiment.
        
        Args:
            req: Service request to simulate the current step
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.logger.log_info("Simulation step execution called...")
        
        if self.target_sequence is None or len(self.target_sequence) == 0:
            self.logger.log_error("No experiment loaded or empty sequence")
            resp.success = False
            return resp
            
        if self.current_step_index >= self.total_steps:
            self.logger.log_info("All steps already completed, load a new experiment")
            resp.success = True
            return resp
        
        if self.sim_complete:
            self.logger.log_info("Simulation for current step already complete. Call /next_step to proceed to next step")
            resp.success = True
            return resp
            
        # Simulate the current step
        success = self.simulate_current_step()
        
        # Report status
        if success:
            remaining = self.total_steps - self.current_step_index
            if remaining > 0:
                if self.exp_execution_active:
                    self.logger.log_info(f"Simulation successful. Execute this step on the real robot manually, then call /next_step. {remaining} steps remaining")
                else:
                    self.logger.log_info(f"Experiment execution is not active. Call /execute_experiment to start the experiment execution flow.")
            else:
                self.logger.log_info("All step simulations completed successfully")
        else:
            self.logger.log_error("Step simulation failed")
            
        resp.success = success
        return resp
    
    def next_step_callback(self, req, resp):
        """
        Signal that the real robot execution is complete and proceed with the next step automatically.
        
        Args:
            req: Service request to signal real robot completion
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.logger.log_info("Manual signal received: Real robot execution completed...")
        
        if not self.sim_complete:
            self.logger.log_info("No simulation has been completed yet for the current step")
            resp.success = False
            return resp
        
        # If in experiment execution, signal the thread to continue
        if self.exp_execution_active:
            self.logger.log_info("Signaling experiment execution to continue to next step...")
            self.wait_for_real_robot.set()
            resp.success = True
            return resp
            
        # If not in experiment execution, advance the step immediately
        self.current_step_index += 1
        self.sim_complete = False
        
        if self.current_step_index >= self.total_steps:
            self.logger.log_info("Final step completed, experiment is finished")
        else:
            self.logger.log_info(f"Advanced to step {self.current_step_index + 1}/{self.total_steps}")
            # Automatically simulate next step
            self.logger.log_info("Automatically simulating next step...")
            self.simulate_current_step()
            
        resp.success = True
        return resp
    
    def stop_exp_execution(self):
        """
        Stop the experiment execution flow if it's running.
        """
        with self.exp_execution_lock:
            self.exp_execution_active = False
            if self.exp_execution_thread and self.exp_execution_thread.is_alive():
                # Check if we're trying to join the current thread
                if self.exp_execution_thread is not threading.current_thread():
                    self.wait_for_real_robot.set()  # Signal thread to exit if waiting
                    self.exp_execution_thread.join(timeout=1.0)
                else:
                    self.logger.log_warn("Cannot join current thread - will terminate naturally")
            self.exp_execution_thread = None
            self.wait_for_real_robot.clear()
    
    def exp_execution_thread_func(self):
        """
        Thread function that handles the experiment execution flow.
        """
        
        while self.exp_execution_active and self.current_step_index < self.total_steps:

            # Simulate current step if not already simulated
            if not self.sim_complete:
                # Check if we can use cached behavior tree
                cached_bt_path = self.get_cached_behavior_tree(self.current_experiment_id, self.current_step_index + 1)
                robot_cached = False

                if cached_bt_path and not self.experiment_changed:
                    self.logger.log_info(f"Using cached behavior tree for step {self.current_step_index + 1}")
                    # Get package path for XML files
                    pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
                    if pkg_path is None:
                        self.logger.log_error("ras_bt_framework package path not found")
                        break
                        
                    real_path = Path(pkg_path) / "xml" / "real.xml"
                    
                    # Copy cached behavior tree to real.xml
                    try:
                        shutil.copy(cached_bt_path, real_path)
                        robot_cached = True
                        self.logger.log_info("Successfully copied cached behavior tree")
                        
                        # Send execution command to real robot
                        self.logger.log_info("Sending execution command to real robot...")
                        self.send_execute_exp_goal()
                        
                        self.sim_complete = True
                    except Exception as e:
                        self.logger.log_error(f"Error copying cached behavior tree: {e}")
                        robot_cached = False
                
                if not robot_cached:
                    # If we can't use cached behavior tree, proceed with normal simulation
                    success = self.simulate_current_step()
                    if not success:
                        self.logger.log_error("Step simulation failed, stopping experiment execution")
                        break
            
            # Wait for real robot execution to complete
            self.logger.log_info(f"Waiting for real robot to complete step {self.current_step_index + 1}...")
            
            # Clear the event
            self.wait_for_real_robot.clear()
            
            # Wait with a timeout and periodic checks
            wait_timeout = 300  # 5 minutes timeout
            polling_interval = 2  # Check every 2 seconds
            elapsed_time = 0
            
            while not self.wait_for_real_robot.is_set() and elapsed_time < wait_timeout and self.exp_execution_active:
                # Limited-time wait allows for periodic status checks
                if self.wait_for_real_robot.wait(polling_interval):
                    self.logger.log_info("Received signal to proceed to next step")
                    break
                
                elapsed_time += polling_interval
                if elapsed_time == 60:  # Call restart_transport_services exactly at 60 seconds
                    self.logger.log_info("60 seconds elapsed, restarting transport services...")
                    self.restart_transport_services()
                    self.stop_exp_execution()
                    
                elif elapsed_time % 30 == 0 and elapsed_time != 60:  # Log status every 30 seconds, except when restarting services
                    self.logger.log_info(f"Still waiting for real robot execution status... ({elapsed_time}s elapsed)")
            
            if not self.wait_for_real_robot.is_set() and self.exp_execution_active:
                self.logger.log_warn(f"Timeout waiting for real robot execution status after {elapsed_time}s")
                # Add a handle condition here
                self.stop_exp_execution()  
            
            if not self.exp_execution_active:
                self.logger.log_info("Experiment execution stopped while waiting for real robot")
                break
            
            # Move to the next step
            self.current_step_index += 1
            self.sim_complete = False
            
            self.save_experiment_state()

            if self.current_step_index >= self.total_steps:
                self.logger.log_info("All steps completed successfully")
            else:
                self.logger.log_info(f"Advanced to step {self.current_step_index + 1}/{self.total_steps}")
                # The main loop will handle the next step in the next iteration
        
        with self.exp_execution_lock:
            self.exp_execution_active = False
            if self.current_step_index >= self.total_steps:
                self.logger.log_info("Experiment execution completed")
                # Save experiment hash with experiment_id and current_hash, but only if not already present
                exp_id = getattr(self, "current_experiment_id", None)
                if exp_id is not None:
                    hashes = self.load_experiment_hashes()
                    if exp_id in hashes:
                        self.logger.log_info(f"Experiment hash for {exp_id} already exists, not recalculating.")
                    else:
                        exp_path = os.path.join(RAS_CONFIGS_PATH, "experiments", f"{exp_id}.yaml")
                        current_hash = self.calculate_file_hash(exp_path)
                        if current_hash:
                            self.save_experiment_hash(exp_id, current_hash)
                            self.logger.log_info(f"Saved experiment hash for {exp_id} after completion.")
                        else:
                            self.logger.log_warn(f"Could not calculate hash for experiment {exp_id} after completion.")
            else:
                self.logger.log_info("Experiment execution stopped")

            self.save_experiment_state()
    
    def execute_experiment_callback(self, req, resp):
        """
        Start the experiment flow that will process all steps in sequence.
        
        Args:
            req: Service request to start the automated flow
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """

        exp_id = req.exepriment_id
        path = os.path.join(RAS_CONFIGS_PATH, "experiments", f"{exp_id}.yaml")
        if not Path(path).exists():
            self.logger.log_error("Experiment Not Found")
            resp.success = False
            return resp
        
        # Save the current experiment ID to a file for log_receiver to access
        current_exp_path = os.path.join(RAS_CONFIGS_PATH, "current_experiment.txt")
        with open(current_exp_path, 'w') as f:
            f.write(exp_id)
        # self.logger.log_info(f"Saved current experiment ID {exp_id} to {current_exp_path}")
        
        # Check if experiment file has changed since last time
        has_changed, current_hash, is_first_time = self.check_experiment_changed(exp_id, path)
        self.current_experiment_hash = current_hash
        self.current_experiment_id = exp_id

        # Log whether the experiment has changed
        if is_first_time:
            self.logger.log_info(f"Experiment {exp_id} is being loaded for the first time")
        elif has_changed:
            self.logger.log_info(f"Experiment {exp_id} has been modified since last run")
        else:
            self.logger.log_info(f"Experiment {exp_id} is unchanged from previous run")
        
        # Store the hash status in the response and class property
        resp.experiment_changed = has_changed
        self.experiment_changed = has_changed or is_first_time

        # Load the experiment data
        self.pose_dict, self.target_sequence = read_yaml_to_pose_dict(path)
        # Register all poses
        if self.pose_dict is not None:
            for pose_name, pose in self.pose_dict.items():
                action_mapping.register_pose(pose_name, pose)
        
        self.current_step_index = 0
        self.total_steps = len(self.target_sequence)
        self.sim_complete = False
       
        self.current_experiment_id = exp_id

        self.save_experiment_state()

        # Stop any ongoing experiment execution before starting a new one
        self.stop_exp_execution()
        
        self.logger.log_info(f"Experiment Loaded with {self.total_steps} steps...")

        # --- Cache Check Logic ---
        self.logger.log_info(f"Checking cache status for experiment '{exp_id}' on robot...")
        cache_status_req = CheckCacheStatus.Request()
        cache_status_req.experiment_id = exp_id

        # Wait for the cache_status service to be available
        if not self.cache_status_client.wait_for_service(timeout_sec=5.0):
            self.logger.log_warn("Cache status service not available on robot. Proceeding without cache check.")
            # If service is not available, proceed with normal execution flow
            self.logger.log_info("Starting the experiment execution (no cache check)...")
            with self.exp_execution_lock:
                 if not self.exp_execution_active:
                     self.exp_execution_active = True
                     self.exp_execution_thread = threading.Thread(target=self.exp_execution_thread_func)
                     self.exp_execution_thread.daemon = True
                     self.exp_execution_thread.start()
            resp.success = True # Indicate success for loading and starting flow
            return resp
            
        # Call the cache_status service
        future = self.cache_status_client.call_async(cache_status_req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0) # Wait for the response

        if future.result() is not None:
            cache_present = future.result().cache_present
            self.logger.log_info(f"Cache status response for '{exp_id}': {cache_present}")

            if cache_present:
                self.logger.log_info(f"Cache found on robot for experiment '{exp_id}'. Robot will execute locally.")
                
                # Prepare response message indicating cache was found
                resp.success = True 
                resp.message = f"Cache found on robot for experiment '{exp_id}'. Robot will execute locally."
                # Do NOT return here, continue to start the execution thread

            else:
                self.logger.log_info(f"Cache not found on robot for experiment '{exp_id}'. Proceeding with normal execution.")
                # Prepare response message indicating cache was not found
                resp.success = True # Indicate success for loading and starting flow
                resp.message = f"Cache not found on robot for experiment '{exp_id}'. Proceeding with normal execution."

            # In both cases (cache found or not found), start the experiment execution flow
            self.logger.log_info("Starting the experiment execution thread...")
            with self.exp_execution_lock:
                if not self.exp_execution_active:
                    self.exp_execution_active = True
                    self.exp_execution_thread = threading.Thread(target=self.exp_execution_thread_func)
                    self.exp_execution_thread.daemon = True
                    self.exp_execution_thread.start()
                else:
                    self.logger.log_info("Experiment execution already running, not starting new thread.")
            
            # Return the prepared response after starting the thread
            return resp

        else:
            self.logger.log_error("Failed to call cache_status service. Proceeding without cache check.")
            # If service call fails, fall back to normal execution flow
            self.logger.log_info("Starting the experiment execution (cache check failed)....") # Corrected log message
            with self.exp_execution_lock:
                 if not self.exp_execution_active:
                     self.exp_execution_active = True
                     self.exp_execution_thread = threading.Thread(target=self.exp_execution_thread_func)
                     self.exp_execution_thread.daemon = True
                     self.exp_execution_thread.start()
            resp.success = True # Indicate success for loading and starting flow
            return resp
