import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ras_common.config.loaders.lab_setup import LabSetup as LabLoader
from ras_resource_lib.managers.asset_manager import AssetManager,AssetType
from ras_resource_lib.types.manipulator.component import ManipulatorComponent
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def generate_launch_description():
    LabLoader.init()
    AssetManager.init()
    robot_component : ManipulatorComponent = AssetManager.get_asset_component(LabLoader.robot_name,AssetType.MANIPULATOR)
    moveit_config = robot_component.moveit_config
    lab_component = AssetManager.get_asset_component(LabLoader.lab_name,AssetType.LAB)
    launch_world = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ros_ign_gazebo'),
            'launch', 'ign_gazebo.launch.py')]),
    launch_arguments=[('gz_args', [f' -r {lab_component}'])]
    )

    # gazebo spawn entity node
    robot_pose = LabLoader.robot_pose
    robot_rot_eulers = euler_from_quaternion((robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w))
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-string', moveit_config.robot_description['robot_description'],
            '-name', robot_component.label,
            '-x', str(robot_pose.position.x),
            '-y', str(robot_pose.position.y),
            '-z', str(robot_pose.position.z),
            '-Y', str(robot_rot_eulers[2]),
        ],
        parameters=[{'use_sim_time': True}],
    )
    # Publish TF
    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    bridge_config = os.path.join(get_package_share_directory("ras_asset_lab_oss_labs"), "camera_bridge_ign", "config.yaml")
    
    ros_gz_bridge = Node(package="ros_gz_bridge", 
            executable="parameter_bridge",
            parameters = [
                {'config_file': bridge_config}],
            )

    return LaunchDescription(
        [
            robot_state_publisher,
            launch_world,
            gazebo_spawn_entity_node,
            ros_gz_bridge
        ]
    )