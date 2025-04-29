#!/usr/bin/env python3



import rclpy
from ras_bt_framework.managers.experiments_service import ExperimentService
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init(args=None)
    expsrv = ExperimentService()
    executor = MultiThreadedExecutor()
    executor.add_node(expsrv)
    executor.add_node(expsrv.batman)

    while rclpy.ok():
        executor.spin_once()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
