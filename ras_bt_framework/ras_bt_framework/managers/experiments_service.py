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
        self.create_service(SetBool, "/test_experiment", self.bt_execution_callback,callback_group=self.my_callback_group)
        self.counter_reset_client = self.create_client(SetBool, '/reset_counter', callback_group=self.my_callback_group)
        self.create_service(LoadExp, "/get_exepriment", self.load_exp, callback_group=self.my_callback_group)
        self.batman = BaTMan()

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
        path = os.path.join(RAS_CONFIGS_PATH,"experiments",f"{exp_id}.yaml")
        if not Path(path).exists():
            self.get_logger().error("Experiment Not Found")
            resp.success = False
            return resp
        # print(path)
        pose_dict, target_pose = read_yaml_to_pose_dict(path)
        self.batman.generate_module_from_keywords(target_pose, pose_dict)
        self.get_logger().info("Experiment Loaded...")
        resp.success = True
        return resp
    
    def bt_execution_callback(self, req, resp):
        """
        Execute the behavior tree for the loaded experiment.
        
        Args:
            req: Service request to execute the behavior tree
            resp: Service response indicating success or failure
            
        Returns:
            The updated service response with success status
        """
        self.get_logger().info("Batman Called ...")
        if isinstance(self.batman.main_module, type(None)):
            self.get_logger().error("Load Experiment First....")
            resp.success = False
            return
        counter_reset = SetBool.Request()
        counter_reset.data = True
        self.counter_reset_client.call_async(counter_reset)
        pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
        if pkg_path is None:
            self.get_logger().error("ras_bt_framework package Path Not Found")
            resp.success = False
            return resp
        path = Path(pkg_path)/"xml"/"sim.xml"
        # behavior = PickObject(self.sequence_list)
        status = self.batman.run_module(path)
        if status in [BTNodeStatus.SUCCESS,BTNodeStatus.IDLE]:
            self.get_logger().info("BT Execution Successful")
        else:
            self.get_logger().info("BT Execution Failed")
            # resp.success = False
            # return resp
        self.get_logger().info("real_bt_generation_started")
        new_module = update_bt(self.batman.main_module)
        btg = BehaviorTreeGenerator(self.batman.alfred)
        btg.feed_root(new_module)
        pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
        if pkg_path is None:
            self.get_logger().error("Package Path Not Found")
            resp.success = False
            return resp
        else:
            bt_path = str(pkg_path)+"/xml/real.xml"
            try:
                btg.generate_xml_trees(bt_path)
            except Exception as e:
                self.get_logger().error(f"Error in BT Generation: {e}")
                resp.success = False
                return resp
        # tree = ET.parse(path)
        # root = tree.getroot()
        # update_xml(root)
        # tree.write(str(pkg_path)+"/xml/real.xml", encoding="utf-8", xml_declaration=True)
        resp.success = True
        return resp
