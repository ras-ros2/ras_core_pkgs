from launch import LaunchDescription
from launch_ros.actions import Node
from ras_common.config.loaders.lab_setup import LabSetup as LabLoader
from ras_resource_lib.managers.asset_manager import AssetManager, AssetType
from ras_resource_lib.types.manipulator.component import ManipulatorComponent
import os

def generate_launch_description():
    app_type = os.getenv("APP_TYPE", "robot")

    AssetManager.init()
    LabLoader.init()

    if app_type == "server":
        # Use asset manager to get the robot component from config
        robot_component: ManipulatorComponent = AssetManager.get_asset_component(
            LabLoader.robot_name, AssetType.MANIPULATOR
        )
        movegroup_name = robot_component.movegroup_name
    else:
        # Use a hardcoded fallback for robot mode
        movegroup_name = "lite6"

    return LaunchDescription([
        Node(
            package='ras_moveit',
            executable='moveit_server',
            name='moveit_server',
            output='screen',
            parameters=[
                {'move_group_name': movegroup_name},
            ]
        ),
    ])

