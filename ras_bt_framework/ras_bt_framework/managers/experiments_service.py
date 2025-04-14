import os
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
        self.my_callback_group = ReentrantCallbackGroup()
        self.create_service(SetBool, "/test_experiment", self.sim_step_callback, callback_group=self.my_callback_group)
        self.create_service(SetBool, "/next_step", self.next_step_callback, callback_group=self.my_callback_group)
        self.create_service(SetBool, "/start_auto_flow", self.start_auto_flow_callback, callback_group=self.my_callback_group)
        self.create_service(SetBool, "/real_robot_done", self.real_robot_done_callback, callback_group=self.my_callback_group)
        self.counter_reset_client = self.create_client(SetBool, '/reset_counter', callback_group=self.my_callback_group)
        self.create_service(LoadExp, "/get_exepriment", self.load_exp, callback_group=self.my_callback_group)
        self.batman = BaTMan()
        
        # Track current step in execution
        self.pose_dict = None
        self.target_sequence = None
        self.current_step_index = 0
        self.total_steps = 0
        self.sim_complete = False
        
        # Threading lock for auto flow
        self.auto_flow_active = False
        self.auto_flow_thread = None
        self.auto_flow_lock = threading.Lock()
        self.wait_for_real_robot = threading.Event()

    def load_exp(self, req, resp):
        """
        Load an experiment configuration and generate behavior modules.
        
        Args:
            req: Service request containing the experiment ID
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        exp_id = req.exepriment_id
        path = os.path.join(RAS_CONFIGS_PATH, "experiments", f"{exp_id}.yaml")
        if not Path(path).exists():
            self.get_logger().error("Experiment Not Found")
            resp.success = False
            return resp
            
        self.pose_dict, self.target_sequence = read_yaml_to_pose_dict(path)
        self.batman.generate_module_from_keywords(self.target_sequence, self.pose_dict)
        
        # Reset step counter when loading new experiment
        self.current_step_index = 0
        self.total_steps = len(self.target_sequence)
        self.sim_complete = False
        
        # Reset auto flow state
        self.stop_auto_flow()
        
        self.get_logger().info(f"Experiment Loaded with {self.total_steps} steps...")
        resp.success = True
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
            self.get_logger().info("All steps completed successfully")
            return True
            
        # Get package path for XML files
        pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
        if pkg_path is None:
            self.get_logger().error("ras_bt_framework package path not found")
            return False
            
        sim_path = Path(pkg_path) / "xml" / "sim.xml"
        real_path = Path(pkg_path) / "xml" / "real.xml"
        
        # Get current step
        current_step = [self.target_sequence[self.current_step_index]]
        step_name = f"Step{self.current_step_index + 1}"
        
        self.get_logger().info(f"Processing step {self.current_step_index + 1}/{self.total_steps}: {current_step}")
        
        # Reset counter for this step
        counter_reset = SetBool.Request()
        counter_reset.data = True
        self.counter_reset_client.call_async(counter_reset)
        
        # Generate module for single step
        step_module = BehaviorModuleSequence()
        self.batman.keyword_module_gen.generate_into(step_module, step_name, current_step)
        
        # Generate XML for simulation
        self.get_logger().info(f"Generating simulation trajectory for step {self.current_step_index + 1}...")
        btg = BehaviorTreeGenerator(self.batman.alfred)
        btg.feed_root(step_module)
        
        try:
            btg.generate_xml_trees(str(sim_path))
        except Exception as e:
            self.get_logger().error(f"Error generating trajectory for simulation: {e}")
            return False
            
        # Run simulation
        self.get_logger().info(f"Running simulation for step {self.current_step_index + 1}...")
        sim_status = self.batman.execute_bt(sim_path)
        
        if sim_status not in [BTNodeStatus.SUCCESS, BTNodeStatus.IDLE]:
            self.get_logger().error(f"Simulation failed for step {self.current_step_index + 1}")
            return False
            
        self.get_logger().info(f"Simulation successful for step {self.current_step_index + 1}")
        
        # Generate real robot XML, but don't execute it yet
        self.get_logger().info(f"Generating real robot trajectory for step {self.current_step_index + 1}...")
        real_module = update_bt(step_module)
        btg = BehaviorTreeGenerator(self.batman.alfred)
        btg.feed_root(real_module)
        
        try:
            btg.generate_xml_trees(str(real_path))
            self.get_logger().info(f"Real robot trajectory generated at {real_path}")
            self.get_logger().info("Execute this on the real robot manually, then call the /real_robot_done service to proceed")
        except Exception as e:
            self.get_logger().error(f"Error generating trajectory for real robot: {e}")
            return False
        
        self.sim_complete = True
        return True
        
    def sim_step_callback(self, req, resp):
        """
        Simulate the current step of the loaded experiment.
        
        Args:
            req: Service request to simulate the current step
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.get_logger().info("Simulation step execution called...")
        
        if self.target_sequence is None or len(self.target_sequence) == 0:
            self.get_logger().error("No experiment loaded or empty sequence")
            resp.success = False
            return resp
            
        if self.current_step_index >= self.total_steps:
            self.get_logger().info("All steps already completed, load a new experiment")
            resp.success = True
            return resp
        
        if self.sim_complete:
            self.get_logger().info("Simulation for current step already complete. Call /next_step or /real_robot_done to proceed to next step")
            resp.success = True
            return resp
            
        # Simulate the current step
        success = self.simulate_current_step()
        
        # Report status
        if success:
            remaining = self.total_steps - self.current_step_index
            if remaining > 0:
                if self.auto_flow_active:
                    self.get_logger().info(f"Simulation successful. Execute this step on the real robot manually, then call /real_robot_done. {remaining} steps remaining")
                else:
                    self.get_logger().info(f"Simulation successful. Execute this step on the real robot manually, then call /next_step. {remaining} steps remaining")
            else:
                self.get_logger().info("All step simulations completed successfully")
        else:
            self.get_logger().error("Step simulation failed")
            
        resp.success = success
        return resp
        
    def next_step_callback(self, req, resp):
        """
        Move to the next step after real robot execution has been manually completed.
        
        Args:
            req: Service request to move to the next step
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.get_logger().info("Next step called...")
        
        if not self.sim_complete:
            self.get_logger().info("Current step simulation not complete yet. Run simulation first.")
            resp.success = False
            return resp
            
        # Advance to the next step
        self.current_step_index += 1
        self.sim_complete = False
        
        if self.current_step_index >= self.total_steps:
            self.get_logger().info("All steps completed successfully")
        else:
            self.get_logger().info(f"Advanced to step {self.current_step_index + 1}/{self.total_steps}. Call /test_experiment to simulate this step.")
            
        resp.success = True
        return resp
    
    def real_robot_done_callback(self, req, resp):
        """
        Signal that the real robot execution is complete and proceed with the next step automatically.
        
        Args:
            req: Service request to signal real robot completion
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.get_logger().info("Real robot execution completed signal received...")
        
        if not self.sim_complete:
            self.get_logger().info("Current step simulation not complete yet. Run simulation first.")
            resp.success = False
            return resp
        
        # If in auto mode, signal the waiting thread to continue
        if self.auto_flow_active:
            self.wait_for_real_robot.set()
            resp.success = True
            return resp
        else:
            # Otherwise, just advance to the next step immediately
            self.current_step_index += 1
            self.sim_complete = False
            
            if self.current_step_index >= self.total_steps:
                self.get_logger().info("All steps completed successfully")
            else:
                self.get_logger().info(f"Advanced to step {self.current_step_index + 1}/{self.total_steps}. Call /test_experiment to simulate this step.")
                
            resp.success = True
            return resp
    
    def stop_auto_flow(self):
        """
        Stop the automated flow if it's running.
        """
        with self.auto_flow_lock:
            self.auto_flow_active = False
            if self.auto_flow_thread and self.auto_flow_thread.is_alive():
                self.wait_for_real_robot.set()  # Signal thread to exit if waiting
                self.auto_flow_thread.join(timeout=1.0)
            self.auto_flow_thread = None
            self.wait_for_real_robot.clear()
    
    def auto_flow_thread_func(self):
        """
        Thread function that handles the automated flow.
        """
        self.get_logger().info("Starting automated experiment flow...")
        
        while self.auto_flow_active and self.current_step_index < self.total_steps:
            # Simulate current step if not already simulated
            if not self.sim_complete:
                success = self.simulate_current_step()
                if not success:
                    self.get_logger().error("Step simulation failed, stopping auto flow")
                    break
            
            # Wait for real robot execution to complete
            self.get_logger().info(f"Waiting for real robot to complete step {self.current_step_index + 1}...")
            self.wait_for_real_robot.clear()
            self.wait_for_real_robot.wait()
            
            if not self.auto_flow_active:
                self.get_logger().info("Auto flow stopped while waiting for real robot")
                break
                
            # Move to next step
            self.current_step_index += 1
            self.sim_complete = False
            
            if self.current_step_index >= self.total_steps:
                self.get_logger().info("All steps completed successfully")
            else:
                self.get_logger().info(f"Advanced to step {self.current_step_index + 1}/{self.total_steps}")
        
        with self.auto_flow_lock:
            self.auto_flow_active = False
            self.get_logger().info("Automated flow completed or stopped")
    
    def start_auto_flow_callback(self, req, resp):
        """
        Start the automated flow that will process all steps in sequence.
        
        Args:
            req: Service request to start the automated flow
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.get_logger().info("Starting automated experiment flow...")
        
        if self.target_sequence is None or len(self.target_sequence) == 0:
            self.get_logger().error("No experiment loaded or empty sequence")
            resp.success = False
            return resp
        
        with self.auto_flow_lock:
            if self.auto_flow_active:
                self.get_logger().info("Automated flow already running")
                resp.success = True
                return resp
                
            # Start the automated flow in a separate thread
            self.auto_flow_active = True
            self.auto_flow_thread = threading.Thread(target=self.auto_flow_thread_func)
            self.auto_flow_thread.daemon = True
            self.auto_flow_thread.start()
            
        resp.success = True
        return resp
