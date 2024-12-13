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

def template_func(asset_name):
    from ras_resource_lib.types.manipulator.config import ManipulatorCfg
    from trajectory_msgs.msg import JointTrajectory
    class MyManipulatorCfg(ManipulatorCfg):
        def __init__(self):
            #things to do before initialization and assignment
            super().__init__(
                label=asset_name,
                movegroup_name="manipulator",
                moveit_config=gen_moveit_config(),
                launch_actions=[],
            )
            
        def execute_trajectory(self,trajectory: JointTrajectory):
            #do something with the trajectory
            pass


    a = MyManipulatorCfg()

    def generate_configuration():
        return MyManipulatorCfg()