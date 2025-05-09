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

from xarm.wrapper import XArmAPI
import rclpy
from rclpy.node import Node
import os
import math
import time
import sys

from std_srvs.srv import SetBool
'''sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))'''
class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.ip = "192.168.1.111"

        self.arm = XArmAPI(self.ip)
        

        self.gripper_srv = self.create_service(SetBool, 'fake_gripper', self.gripper_callback)
        
        self.restart_srv = self.create_service(SetBool, 'restart_arm', self.restart_callback)


    def gripper_callback(self, request, response):
        print("GRIPPER CALLED")
        if request.data == True:
            time.sleep(2)
            self.arm.close_lite6_gripper(True)
            time.sleep(2)
        else:
            time.sleep(2)
            self.arm.open_lite6_gripper(True)
            time.sleep(2)
        response.success = True

        return response
    
    def restart_callback(self, request, response):
        print("ARM RESART CALLED")
        if request.data == True:
            time.sleep(10)
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(state=0)
            self.arm.reset()
        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperControl()
    rclpy.spin(gripper_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

