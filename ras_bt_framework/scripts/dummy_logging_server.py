#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty

class ExecutionCount():
    def __init__(self):
        self.count = 0
    def increment(self):
        self.count += 1
        print(f"count is {self.count}")

class DummyLoggingServer(Node):
    def __init__(self, function):
        super().__init__("dummy_logging_server")
        self.create_service(SetBool, "dummy_logging_server", self.log_callback)
        self.send_client = self.create_client(Empty, "/start_logging")
        assert(callable(function))
        self.function = function
        

    def log_callback(self, req: SetBool.Request, res: SetBool.Response):
        # TODO (Sachin): Add logging logic later
        self.function()
        empty = Empty.Request()
        self.send_client.call_async(empty)
        res.success = True
        return res
    
def main(args=None):
    rclpy.init(args=args)
    execution_count = ExecutionCount()
    dummy_logging_server = DummyLoggingServer(execution_count.increment)
    rclpy.spin(dummy_logging_server)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
