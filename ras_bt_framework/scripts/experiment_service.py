#!/usr/bin/env python3

"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

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
