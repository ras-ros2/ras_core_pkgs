#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from ras_transport.interfaces.TransportWrapper import TransportMQTTSubscriber, TransportServiceServer, TransportServiceClient
from ras_common.config.loaders.lab_setup import LabSetup
from ras_interfaces.srv import JointReq
from typing import Dict
from enum import Enum
from std_srvs.srv import SetBool
import json
import time
from ras_logging.ras_logger import RasLogger
class TransportCommands(Enum):
    HOME = "home"
    SYNC = "sync"

class TransportRobotService(Node):
    def __init__(self):
        super().__init__('transport_robot_service')
        self.logger = RasLogger()
        self.server_transport_server = TransportServiceServer("server_transport", self.execute_request)
        
        self.sync_robot_client = TransportServiceClient("sync_robot")
        self.connect_aws()
        time.sleep(2) # wait for 2 seconds
        self.first_sync_robot()

    def first_sync_robot(self):
        payload = {
            "sync_robot": True
        }
        req = json.dumps(payload)
        res = self.sync_robot_client.call(req)
        message = json.loads(res)
        if message["success"]:
            print("Initial Sync Robot Started ")
            result = self.execute_request(TransportCommands.SYNC.value.encode("utf-8"))
            payload = json.loads(result)
            if (payload["success"]):
                print("Initialized sync robot successfully")
            else:
                print("Initialized sync robot not happend")

        else:
            print("Initial Sync Robot Not Happend")

    def connect_aws(self):
        self.server_transport_server.connect_with_retries()
        self.sync_robot_client.connect_with_retries()
    
    

    def execute_request(self, message):
        payload = message.decode("utf-8")
        if payload == TransportCommands.HOME.value:
            self.logger.log_info("Executing Home command")
            req : JointReq.Request = JointReq.Request()
            LabSetup.init()
            home_joint_state: Dict[str, float|int] = LabSetup.conf.robot.home_joint_state
            req.joints.header.stamp = self.get_clock().now().to_msg()
            req.joints.name = list(home_joint_state.keys())
            req.joints.position = [float(value) for value in home_joint_state.values()]
            self.logger.log_info(f"Home joint state: {home_joint_state}")
            client = self.create_client(JointReq, '/move_to_joint_states')
            while not client.wait_for_service(timeout_sec=1.0):
                self.logger.log_info('Service not available, waiting again...')
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response: JointReq.Response = future.result()
                if response.success:
                    self.logger.log_info("Home position reached successfully")
                    self.execute_request(TransportCommands.SYNC.value.encode("utf-8"))
                else:
                    self.logger.log_info("Home position doesn't reached")
            else:
                self.logger.log_error(f"Service call failed: {future.exception()}")
        elif payload == TransportCommands.SYNC.value:
            self.logger.log_info("Executing Sync command")
            req : SetBool.Request = SetBool.Request()
            req.data = True
            client = self.create_client(SetBool, '/dummy_logging_server')
            while not client.wait_for_service(timeout_sec=1.0):
                self.logger.log_info('Service dummy_logging_server not available, waiting again...')
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                response: SetBool.Response = future.result()
                if response.success:
                    self.logger.log_info("Sync command executed successfully")
                else:
                    self.logger.log_info("Sync command execution failed")
            else:
                self.logger.log_error(f"Service call failed: {future.exception()}")
        else:
            self.logger.log_info("Invalid command")

        payload = {
            "success": True
        }
        return json.dumps(payload)

def main(args=None):
    rclpy.init(args=args)
    node = TransportRobotService()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.server_transport_server.loop()
            node.sync_robot_client.loop()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and disconnect
        node.server_transport_server.disconnect()
        node.sync_robot_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
