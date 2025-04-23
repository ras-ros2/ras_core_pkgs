import os
import yaml
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
from ras_common.globals import RAS_CONFIGS_PATH
import time
from ..behaviors.modules import BehaviorModuleSequence
import threading
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
from ras_interfaces.action import ExecuteExp
import subprocess
from ras_logging.ras_logger import RasLogger

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
    
    def __init__(self):
        """
        Initialize the ExperimentService node.
        
        This sets up the necessary services and clients for experiment management.
        """
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
        
        # Threading lock for experiment execution
        self.exp_execution_active = False
        self.exp_execution_thread = None
        self.exp_execution_lock = threading.Lock()
        self.wait_for_real_robot = threading.Event()
    
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
        
        # Get current step
        current_step = [self.target_sequence[self.current_step_index]]
        step_name = f"Step{self.current_step_index + 1}"
        
        self.logger.log_info(f"Processing step {self.current_step_index + 1}/{self.total_steps}: {current_step}")
        
        # Reset counter for this step
        counter_reset = SetBool.Request()
        counter_reset.data = True
        self.counter_reset_client.call_async(counter_reset)
        
        # Generate module for single step
        step_module = BehaviorModuleSequence()
        self.batman.keyword_module_gen.generate_into(step_module, step_name, current_step)
        
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
            self.logger.log_info("Sending execution command to real robot...")
            
            # Execute experiment on real robot using ROS2 action
            self.send_execute_exp_goal()
            
        except Exception as e:
            self.logger.log_error(f"Error generating trajectory for real robot: {e}", e)
            return False
        
        self.sim_complete = True
        return True
    
    def send_execute_exp_goal(self):
        """
        Send a goal to the /execute_exp action server to run the experiment on the real robot.
        """
        # Wait for action server
        if not self.execute_exp_client.wait_for_server(timeout_sec=1.0):
            self.logger.log_info("Action server not available, using command line approach instead")
            # Fall back to using command line
            try:
                cmd = "ros2 action send_goal /execute_exp ras_interfaces/action/ExecuteExp {}"
                self.logger.log_info(f"Executing command: {cmd}")
                # Execute the command in a non-blocking way
                subprocess.Popen(cmd, shell=True)
                self.logger.log_info("Command sent to execute experiment on real robot")
            except Exception as e:
                self.logger.log_error(f"Failed to execute command: {e}", e)
            return
        
        # Create and send goal
        goal_msg = ExecuteExp.Goal()
        self.logger.log_info("Sending goal to execute experiment on real robot")
        
        # Send the goal asynchronously
        send_goal_future = self.execute_exp_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
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
            
        self.logger.log_info("Goal accepted")
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
            
            # If experiment execution is active, directly set the event
            if self.exp_execution_active and self.sim_complete:
                self.logger.log_info("Setting wait event based on action result")
                self.wait_for_real_robot.set()
                
        except Exception as e:
            self.logger.log_error(f"Error in get_result_callback: {e}", e)
    
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
                self.wait_for_real_robot.set()  # Signal thread to exit if waiting
                self.exp_execution_thread.join(timeout=1.0)
            self.exp_execution_thread = None
            self.wait_for_real_robot.clear()
    
    def exp_execution_thread_func(self):
        """
        Thread function that handles the experiment execution flow.
        """
        
        while self.exp_execution_active and self.current_step_index < self.total_steps:
            # Simulate current step if not already simulated
            if not self.sim_complete:
                success = self.simulate_current_step()
                if not success:
                    self.logger.log_error("Step simulation failed, stopping experiment execution")
                    break
                
                # Note: We don't need to explicitly execute on real robot here as it's now done in simulate_current_step()
            
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
                if elapsed_time % 30 == 0:  # Log every 30 seconds
                    self.logger.log_info(f"Still waiting for real robot execution status... ({elapsed_time}s elapsed)")
            
            if not self.wait_for_real_robot.is_set() and self.exp_execution_active:
                self.logger.log_warn(f"Timeout waiting for real robot execution status after {elapsed_time}s")
            
            if not self.exp_execution_active:
                self.logger.log_info("Experiment execution stopped while waiting for real robot")
                break
                
            # Move to next step
            self.current_step_index += 1
            self.sim_complete = False
            
            if self.current_step_index >= self.total_steps:
                self.logger.log_info("All steps completed successfully")
            else:
                self.logger.log_info(f"Advanced to step {self.current_step_index + 1}/{self.total_steps}")
                # Automatically simulate next step immediately
                self.logger.log_info("Automatically simulating next step...")
                success = self.simulate_current_step()
                if not success:
                    self.logger.log_error("Next step simulation failed, stopping experiment execution")
                    break
        
        with self.exp_execution_lock:
            self.exp_execution_active = False
            if self.current_step_index >= self.total_steps:
                self.logger.log_info("Experiment execution completed")
            else:
                self.logger.log_info("Experiment execution stopped")
    
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
        self.logger.log_info(f"Saved current experiment ID {exp_id} to {current_exp_path}")
        
        self.pose_dict, self.target_sequence = read_yaml_to_pose_dict(path)
        self.batman.generate_module_from_keywords(self.target_sequence, self.pose_dict)
        
        self.current_step_index = 0
        self.total_steps = len(self.target_sequence)
        self.sim_complete = False
        
        self.stop_exp_execution()  
        
        self.logger.log_info(f"Experiment Loaded with {self.total_steps} steps...")
        
        self.logger.log_info("Starting experiment flow...")
        
        if self.target_sequence is None or len(self.target_sequence) == 0:
            self.logger.log_error("No experiment loaded or empty sequence")
            resp.success = False
            return resp
        
        with self.exp_execution_lock:
            if self.exp_execution_active:
                self.logger.log_info("Experiment execution already running")
                resp.success = True
                return resp
                
            # Start the experiment execution flow in a separate thread
            self.exp_execution_active = True
            self.exp_execution_thread = threading.Thread(target=self.exp_execution_thread_func)
            self.exp_execution_thread.daemon = True
            self.exp_execution_thread.start()
            
        resp.success = True
        return resp
