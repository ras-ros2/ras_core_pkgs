#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import json
import time
from ras_interfaces.action import ExecuteExp
from std_srvs.srv import SetBool
from rclpy.action import ActionServer
# from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from ras_interfaces.msg import Instruction
import ast
from trajectory_msgs.msg import JointTrajectory
# from awscrt import mqtt 
import json
# from connection_helper import ConnectionHelper
from ras_interfaces.srv import SetPath
from ras_interfaces.action import ExecuteExp
from ras_transport.interfaces.TransportWrapper import TransportMQTTPublisher, TransportMQTTSubscriber, TransportServiceClient
from ras_transport.interfaces.TransportWrapper import TransportFileClient
import os
import zipfile
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_logging.ras_logger import RasLogger
from rclpy.executors import MultiThreadedExecutor
from ras_common.globals import RAS_CONFIGS_PATH, RAS_CACHE_PATH

# Set APP_TYPE
APP_TYPE = os.environ.get("APP_TYPE", "server")  # fallback default
if APP_TYPE == "server":
    from rclpy.action.server import ServerGoalHandle

class LinkHandler(Node):
    """
    A ROS2 node that handles sending experiment data to the robot.

    This node is responsible for:
    1. Receiving experiment execution requests
    2. Packaging experiment data into zip files
    3. Sending the data to the robot via MQTT
    4. Managing the connection with AWS IoT

    The node can operate in two modes:
    - Server mode: Packages and sends experiment data to the robot
    - Robot mode: Receives and processes experiment data
    """

    def __init__(self):
        """
        Initialize the LinkHandler node.

        Sets up:
        1. MQTT clients for communication
        2. Action server for experiment execution
        3. File client for data transfer
        4. AWS IoT connection
        """
        super().__init__('link_sender')
        self.logger = RasLogger()

        my_callback_group = ReentrantCallbackGroup()

        self.declare_parameter("path_for_config", "")
        self.declare_parameter("discover_endpoints", False)
        self.file_client = TransportFileClient()
        self.send_client = ActionServer(self, ExecuteExp, "/execute_exp", self.send_callback, callback_group=my_callback_group)

        self.mqtt_sub_response_flag = False
        self.mqtt_sub_msg = None
        self.remote_bt_client = TransportServiceClient("remote_bt")
        self.remote_cache_status_client = TransportServiceClient("cache_status")
        self.connect_to_aws()
        self.experiment_name = None

    def connect_to_aws(self):
        self.remote_bt_client.connect_with_retries()
        self.remote_cache_status_client.connect_with_retries()
        self.file_client.connect_with_retries()


    def send_callback(self, goal_handle):
        """
        Callback function for the /execute_exp action server.

        This function:
        1. Reads the current experiment name
        2. Gets the experiment hash from the goal
        3. Creates a zip file of the experiment data
        4. Sends the data to the robot
        5. Returns the execution result

        Args:
            goal_handle (ServerGoalHandle): The goal handle containing the experiment hash

        Returns:
            ExecuteExp.Result: Result indicating success/failure of the operation
        """
        self.logger.log_info("Starting Real Arm.....")

        # Read experiment name from current_experiment.txt
        current_exp_path = os.path.join(RAS_CONFIGS_PATH, "current_experiment.txt")
        try:
            with open(current_exp_path, 'r') as f:
                self.experiment_name = f.read().strip()
        except Exception as e:
            self.logger.log_error(f"Could not read experiment name from {current_exp_path}: {e}")
            goal_handle.abort()
            return ExecuteExp.Result(success=False)

        # Get the hash from the goal message
        hash_id = goal_handle.request.hash_id if hasattr(goal_handle.request, 'hash_id') else "no_hash"
        self.logger.log_info(f"Received hash_id from goal: {hash_id}")

        if APP_TYPE == "server":
            self.logger.log_info(f"APP_TYPE is Server.")
            zip_file_path = self.zip_xml_directory(self.experiment_name, hash_id)
        else:  # robot
            self.logger.log_info(f"APP_TYPE is Robot.")
            pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
            zip_file_path = os.path.join(str(pkg_path), "xml", "xml_directory.zip")

        result = ExecuteExp.Result()

        if not zip_file_path or not os.path.exists(zip_file_path):
            self.logger.log_error("Zip file not found or created")
            result.success = False
            if APP_TYPE == "server":
                goal_handle.abort()
        else:
            # Include hash_id in the response
            resp: str = self.send_zip_file_path(zip_file_path, hash_id)
            data = json.loads(resp)
            result.success = data.get("status")
            goal_handle.succeed()

        return result

    def send_zip_file_path(self, zip_file_path, hash_id) -> str:
        """
        Sends a zip file to the robot and triggers its execution.

        This function:
        1. Uploads the zip file using the file client
        2. Sends a message to the robot to execute the experiment
        3. Returns the execution status

        Args:
            zip_file_path (str): Path to the zip file to send
            hash_id (str): Hash ID of the experiment

        Returns:
            str: JSON string containing the execution status
        """
        request = SetPath.Request()
        request.path = zip_file_path

        if APP_TYPE == "server":
            if self.experiment_name is None:
                 self.logger.log_error("Experiment name is not set.")
                 return json.dumps({"status": False})
            zip_filename = f"xml_{self.experiment_name}_{hash_id}.zip"
        else:  # robot
            zip_filename = "xml_directory.zip"

        result = self.file_client.upload(zip_file_path, zip_filename)
        if result:
            return self.remote_bt_client.call(zip_filename)
        else:
            return json.dumps({"status": False})


    def zip_xml_directory(self, experiment_name, hash_id):
        """
        Creates a zip file containing the experiment data.

        This function:
        1. Locates the experiment data in the cache
        2. Creates a zip file with the experiment data
        3. Handles different paths for server and robot modes

        Args:
            experiment_name (str): Name of the experiment
            hash_id (str): Hash ID of the experiment

        Returns:
            str: Path to the created zip file, or empty string if failed
        """
        cache_path = os.path.join(RAS_CACHE_PATH, "server")
        cache_exp_path = os.path.join(cache_path, experiment_name, hash_id)
        # pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")

        if cache_exp_path is None:
            self.logger.log_error("Could not find the path for experiment cache")
            return ""

        # if pkg_path is None:
        #     self.logger.log_error("Could not find the package path for ras_bt_framework")
        #     return ""

        traj_dir_path = os.path.join(cache_exp_path, "trajectory")
        # xml_dir_path = os.path.join(pkg_path, "xml")
        
        # Server stores zip in a separate "zip" folder; Robot stores it inside the xml folder
        if APP_TYPE == "server":
            zip_dir_path = os.path.join(cache_exp_path)
            zip_file_path = os.path.join(zip_dir_path, f"xml_{experiment_name}_{hash_id}.zip")
        else:  # robot
            zip_file_path = os.path.join(traj_dir_path, "xml_directory.zip")

        # Clean up any existing zip file (especially important on server)
        if os.path.exists(zip_file_path):
            os.remove(zip_file_path)

        # Create the new zip archive
        with zipfile.ZipFile(zip_file_path, 'w') as zipf:
            for root, dirs, files in os.walk(traj_dir_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    arcname = os.path.relpath(file_path, start=traj_dir_path)
                    zipf.write(file_path, arcname)

        return zip_file_path

def main(args=None):
    rclpy.init(args=args)
    link_handler = LinkHandler()
    # Create a multithreaded executor with more threads for better performance
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(link_handler)
    try:
        # Set higher thread priority for this process if possible
        try:
            import os
            os.nice(-10)  # Try to increase process priority (requires root)
        except (ImportError, PermissionError, OSError):
            link_handler.logger.log_warn("Could not set process priority")
        # Process callbacks with the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        link_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

