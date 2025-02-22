from launch import LaunchDescription
from launch_ros.actions import Node
from ras_common.config.loaders.ras_config import RasObject

def generate_launch_description():
    RasObject.init()
    file_server_ip: str = RasObject.ras.transport.file_server.ip
    mqtt_broker_ip: str = RasObject.ras.transport.mqtt.ip

    nodes = [
        Node(
            package='ras_transport',
            executable='log_receiver.py',
            name='log_receiver',
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
    ]
    
    if mqtt_broker_ip in {"localhost", "127.0.0.1", "0.0.0.0"}:
        nodes.append(
            Node(
                package='ras_transport',
                executable='file_server.py',
                name='file_server',
                output='screen'
            )
        )
    
    if file_server_ip in {"localhost", "127.0.0.1", "0.0.0.0"}:
        nodes.append(
             Node(
                package='ras_transport',
                executable='mqtt_broker.py',
                name='mqtt_broker',
                output='screen'
            ),
        )
    
    return LaunchDescription(nodes)
