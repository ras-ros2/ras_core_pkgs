from launch import LaunchDescription
from launch_ros.actions import Node
from ras_common.config.loaders.ras_config import RasObject
from ras_common.socket_utils import check_if_bindable
from ras_logging.ras_logger import RasLogger

def generate_launch_description():
    RasObject.init()
    logger = RasLogger()
    file_srv_cfg = RasObject.ras.transport.file_server
    mqtt_brkr_cfg = RasObject.ras.transport.mqtt
    

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
    if not file_srv_cfg.use_external:
        if check_if_bindable(file_srv_cfg.ip, file_srv_cfg.port):
            nodes.append(
                Node(
                    package='ras_transport',
                    executable='file_server.py',
                    name='file_server',
                    output='screen'
                )
            )
        else:
            logger.log_error(f"ERROR!!: File server not bindable at {file_srv_cfg.ip}:{file_srv_cfg.port}")
            exit(1)
    else:
        logger.log_info("Using external file server")
    if not mqtt_brkr_cfg.use_external:
        if check_if_bindable(mqtt_brkr_cfg.ip, mqtt_brkr_cfg.port):
            nodes.append(
                Node(
                    package='ras_transport',
                    executable='mqtt_broker.py',
                    name='mqtt_broker',
                    output='screen'
                ),
            )
        else:
            logger.log_error(f"ERROR!!: MQTT broker not bindable at {mqtt_brkr_cfg.ip}:{mqtt_brkr_cfg.port}")
            exit(1)
    else:
        logger.log_info("Using external MQTT broker")
    return LaunchDescription(nodes)
