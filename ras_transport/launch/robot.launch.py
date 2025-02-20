from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ras_transport',
            executable='file_server.py',
            name='file_server',
            output='screen'
        ),
        Node(
            package='ras_transport',
            executable='iot_receiver.py',
            name='iot_receiver',
            output='screen'
        ),
        Node(
            package='ras_transport',
            executable='log_sender.py',
            name='log_sender',
            output='screen'
        ),
        Node(
            package='ras_transport',
            executable='transport_robot_service.py',
            name='transport_robot_service',
            output='screen'
        )
    ])
