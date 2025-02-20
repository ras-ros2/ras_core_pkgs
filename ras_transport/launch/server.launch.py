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
            executable='log_receiver.py',
            name='log_receiver',
            output='screen'
        ),
        Node(
            package='ras_transport',
            executable='mqtt_broker.py',
            name='mqtt_broker',
            output='screen'
        ),
        Node(
            package='ras_transport',
            executable='iot_sender.py',
            name='iot_sender',
            output='screen'
        ),
        Node(
            package='ras_transport',
            executable='transport_server_service.py',
            name='transport_server_service',
            output='screen'
        )
    ])
