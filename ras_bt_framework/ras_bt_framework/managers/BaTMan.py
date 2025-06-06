from ..generators.behavior_tree_generator import BehaviorTreeGenerator
from .primitive_action_manager import PrimitiveActionManager
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper, single_joint_state
from ras_bt_framework.generators.keywords_module_generator import KeywordModuleGenerator
from ras_bt_framework.behaviors.modules import BehaviorModuleSequence
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from ras_interfaces.action import BTInterface
from ras_interfaces.srv import PrimitiveExec, ReportRobotState
from ras_interfaces.msg import BTNodeStatus
from pathlib import Path
import time
from ras_logging.ras_logger import RasLogger

class BaTMan(Node):
    """
    A ROS2 node for managing behavior tree execution and primitive actions.
    
    This class provides functionality to generate and execute behavior modules
    based on keywords and pose targets, and manages the interaction with the behavior tree executor.
    
    Attributes:
        alfred (PrimitiveActionManager): Manager for handling primitive actions.
        manager (BehaviorTreeGenerator): Generator for behavior trees.
        _action_client (ActionClient): Client for executing behavior tree actions.
        target_pose_map (TargetPoseMap): Map of registered poses and their configurations.
        keyword_module_gen (KeywordModuleGenerator): Generator for keyword-based modules.
        main_module (BehaviorModuleSequence): Main sequence of behavior modules.
        tick_cli (Client): Client for ticking the behavior tree.
        loop_rate (Rate): Rate for controlling the execution loop.
        session_started (bool): Flag indicating if the session has started.
    """
    
    def __init__(self):
        """
        Initialize the BaTMan node.
        
        This sets up the necessary clients and generators for behavior tree execution.
        """
        super().__init__("batman")
        self.logger = RasLogger()
        self.alfred = PrimitiveActionManager(self)
        self.logger.log_info("Node Init")

        self.manager = BehaviorTreeGenerator(self.alfred)
        self._action_client = ActionClient(self, BTInterface, "bt_executor")
        self.target_pose_map = TargetPoseMap()
        self.keyword_module_gen = KeywordModuleGenerator()
        self.keyword_module_gen.register({
            "move2pose": self.target_pose_map.move2pose_module,
            "Move": self.target_pose_map.move2pose_module,
            "Pick": self.target_pose_map.pick_module,
            "Place": self.target_pose_map.place_module,
            "PlaceObject": self.target_pose_map.place_object_module,
            "PickObject": self.target_pose_map.pick_object_module,
            "PickFront": self.target_pose_map.pick_front_module,
            "PickRight": self.target_pose_map.pick_right_module,
            "PickLeft": self.target_pose_map.pick_left_module,
            "PickRear": self.target_pose_map.pick_rear_module,
            "rotate": rotate,
            "gripper": gripper,
            "move2pose_sequence": self.target_pose_map.move2pose_sequence_module,
            "single_joint_state": single_joint_state,
        })
        self.main_module = BehaviorModuleSequence()
        self.tick_cli = self.create_client(PrimitiveExec, '/bt_tick')
        self.loop_rate = self.create_rate(10)
        self.session_started = False

        # Created a client for /report_robot_state
        self.report_robot_state_client = self.create_client(ReportRobotState, '/report_robot_state')

    def generate_module_from_keywords(self, keywords: list, pose_targets: dict):
        """
        Generate behavior modules from a list of keywords and pose targets.
        
        Args:
            keywords (list): List of keywords representing actions.
            pose_targets (dict): Dictionary mapping pose names to their configurations.
        """
        for _k, _v in pose_targets.items():
            self.target_pose_map.register_pose(_k, _v)
        self.main_module = self.keyword_module_gen.generate("MainModule", keywords)
    
    def send_goal(self, path: str):
        """
        Send a goal to execute the behavior tree at the specified path.
        
        Args:
            path (str): Path to the behavior tree XML file.
        """
        goal_msg = BTInterface.Goal()
        goal_msg.bt_path = path
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: rclpy.Future):
        """
        Callback function for handling the response of a goal request.
        
        Args:
            future (rclpy.Future): Future object containing the goal handle.
        """
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.logger.log_info('Goal rejected :(')
            return
        # self.logger.log_info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.session_started = True

    def get_result_callback(self, future: rclpy.Future):
        """
        Callback function for handling the result of the behavior tree execution.
        
        Args:
            future (rclpy.Future): Future object containing the execution result.
        """
        result: BTInterface.Result = future.result()
        # self.logger.log_info('Result: {0}'.format(result.status))
    
    def feedback_callback(self, feedback_msg):
        """
        Callback function for handling feedback during behavior tree execution.
        
        Args:
            feedback_msg: Feedback message from the action server.
        """
        feedback: BTInterface.Feedback = feedback_msg.feedback
        self.logger.log_info('Received feedback: {0} {1}'.format(feedback.status, feedback.behavior_stack))
    
    def tick_loop(self):
        """
        Main loop for ticking the behavior tree execution.
        
        Returns:
            int: Final status of the behavior tree execution.
        """
        req = PrimitiveExec.Request()
        status = BTNodeStatus.IDLE
        while rclpy.ok():
            if status == BTNodeStatus.IDLE:
                self.logger.log_info("Sending tick")
                future = self.tick_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                resp: PrimitiveExec.Response = future.result()
                status = resp.status.data
                
            # Map BTNodeStatus to robot state string
            if status == BTNodeStatus.SUCCESS:
                robot_state = "idle"
            elif status == BTNodeStatus.FAILURE:
                robot_state = "error"
            elif status == BTNodeStatus.RUNNING:
                robot_state = "running"
            elif status == BTNodeStatus.IDLE:
                robot_state = "idle"
            else:
                robot_state = "unknown"

            # Report robot state after each tick
            if self.report_robot_state_client.wait_for_service(timeout_sec=1.0):
                req_state = ReportRobotState.Request()
                req_state.state = robot_state
                req_state.details = f"BTNodeStatus: {status}"
                future_state = self.report_robot_state_client.call_async(req_state)
                # Optionally, do not wait for response (fire-and-forget)
            else:
                self.logger.log_warn("/report_robot_state service not available.")

            if status in [BTNodeStatus.FAILURE, BTNodeStatus.SKIPPED, BTNodeStatus.SUCCESS, BTNodeStatus.IDLE]:
                break
            self.loop_rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.1)
        return status
    
    def execute_bt(self, bt_path: str | Path):
        """
        Execute a behavior tree by sending it as a goal.
        
        Args:
            bt_path (str | Path): Path to the behavior tree XML file.
        
        Returns:
            int: Final execution status of the behavior tree.
        """
        bt_path = str(bt_path)
        self.logger.log_info("Executing BT at path: {0}".format(bt_path))
        self.send_goal(bt_path)
        time.sleep(0.1)
        return self.tick_loop()

    def run_module(self, path: Path, behavior: BehaviorModuleSequence = None):
        """
        Run a behavior module by generating and executing the behavior tree.
        
        Args:
            path (Path): Path to save the generated behavior tree.
            behavior (BehaviorModuleSequence, optional): The behavior module to execute. Defaults to the main module.
        
        Raises:
            ValueError: If an invalid behavior module is supplied.
        """
        if not isinstance(behavior, BehaviorModuleSequence):
            if not isinstance(self.main_module, BehaviorModuleSequence):
                raise ValueError("Invalid module supplied")
            behavior = self.main_module
        self.manager.feed_root(behavior)
        self.manager.verify_sanity()
        path = Path(path)
        bt_path = str(path.resolve().absolute())
        self.manager.generate_xml_trees(bt_path)
        return self.execute_bt(bt_path)