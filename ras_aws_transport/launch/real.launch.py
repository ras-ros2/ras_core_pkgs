from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ras_aws_transport',
            executable='ras_aws_transport_node',
            name='ras_aws_transport_node',
            output='screen'
        ),
        Node(
            package='ras_aws_transport',
            executable='aws_mqtt_bridge_node',
            name='aws_mqtt_bridge_node',
            output='screen'
        ),
        Node(
            package='ras_aws_transport',
            executable='aws_iot_core_bridge_node',
            name='aws_iot_core_bridge_node',
            output='screen'
        )
    ])
