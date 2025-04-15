#!/usr/bin/env python3

import rclpy
from ras_interfaces.srv import GripperLog, TrajLog, StatusLog
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
from rclpy.node import Node

class LoggingManager(Node):
    def __init__(self):
        super().__init__("logging_manager")

        self.get_logger().info("LOGGING MANAGER_STARTED")
        self.my_callback_group = ReentrantCallbackGroup()

        self.current_traj = 0
        self.gripper_status = False
        self.traj_status = "SUCCESS"
        
        self.create_service(TrajLog, "/traj_status", self.update_traj_status, callback_group=self.my_callback_group)
        self.create_service(GripperLog, "/gripper_status", self.update_gripper_status, callback_group=self.my_callback_group)
        self.create_service(Empty, "/start_logging", self.start_logging, callback_group=self.my_callback_group)

        self.logging_client = self.create_client(StatusLog, "/send_logging", callback_group=self.my_callback_group)


    def update_traj_status(self, req, resp):

        self.get_logger().info("Trajectory Status Updated")

        self.current_traj = req.current_traj
        self.traj_status = req.traj_status

        resp.success = True

        return resp
    
    def update_gripper_status(self, req, resp):

        self.get_logger().info("Gripper Status Updated")

        self.gripper_status = req.gripper_status
        
        resp.success = True

        return resp

    def start_logging(self, req, resp):

        self.get_logger().info("Status Send to LOG SENDER")

        log_data = StatusLog.Request()
        log_data.traj_status = self.traj_status
        log_data.current_traj = self.current_traj
        log_data.gripper_status = self.gripper_status
        
        self.logging_client.call_async(log_data)

        return resp

def main():
    rclpy.init(args=None)
    logging_node = LoggingManager()
    while rclpy.ok():
        rclpy.spin_once(logging_node)
    logging_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


