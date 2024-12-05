from launch import LaunchDescription
from launch_ros.actions import Node
from oss_common.lab_loader import LabLoader
from oss_resource_lib.managers.asset_manager import AssetManager,AssetType
from oss_resource_lib.types.manipulator.component import ManipulatorComponent


def generate_launch_description():
   AssetManager.init()
   LabLoader.init()
   robot_component : ManipulatorComponent = AssetManager.get_asset_component(LabLoader.robot_name,AssetType.MANIPULATOR)
   return LaunchDescription([
      Node(
        package='oss_moveit',
        # namespace='moveit_server',
        executable='moveit_server',
        name='moveit_server',
        output='screen',
        parameters=[
            {'move_group_name': robot_component.movegroup_name},
        ]
    ),

   ])