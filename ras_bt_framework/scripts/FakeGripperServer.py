#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from ras_interfaces.srv import GripperLog
from rclpy.callback_groups import ReentrantCallbackGroup
from ras_logging.ras_logger import RasLogger
from std_msgs.msg import Bool  # For publishing the gripper status as a Bool message

class FakeGripperServer(Node):
    def __init__(self):
        super().__init__("fake_gripper_node")
        self.logger = RasLogger()
        self.my_callback_group = ReentrantCallbackGroup()

        self.logger.log_info("Node Init")
        self.create_service(SetBool, "/fake_gripper", self.gripper_callback, callback_group=self.my_callback_group)
        self.gripper_client = self.create_client(GripperLog, "/gripper_status", callback_group=self.my_callback_group)

        # A publisher to publish gripper status
        self.gripper_status_publisher = self.create_publisher(Bool, '/gripper_status', 10)

    def gripper_callback(self, req, resp):

        gripper_data = GripperLog.Request()
        if req.data == True:
            self.logger.log_info("Gripper ON")
            gripper_data.gripper_status = True
        else:
            self.logger.log_info("Gripper OFF")
            gripper_data.gripper_status = False
        
        self.gripper_client.call_async(gripper_data)

        # Publish the gripper status to /gripper_status topic
        gripper_status_msg = Bool()
        gripper_status_msg.data = gripper_data.gripper_status
        self.gripper_status_publisher.publish(gripper_status_msg)

        resp.success = True

        return resp

def main():
    rclpy.init(args=None)
    node = FakeGripperServer()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

