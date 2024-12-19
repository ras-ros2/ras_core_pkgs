from ..config_loaders.RasConfigLoader import RasConfigLoader
from .implementations.DefaultTransport import FtpServer, FtpClient, MqttPublisher,MqttSubscriber
import os
from pathlib import Path

class TransportFTPServer(object):
    def __init__(self, name: str) -> None:
        RasConfigLoader.init()
        serve_path = Path(os.environ["RAS_APP_PATH"] + "/serve")
        serve_path.mkdir(parents=True, exist_ok=True)
        self.name = name
        ftp_conf = RasConfigLoader.ras.transport.ftp
        if not hasattr(ftp_conf, self.name):
            raise Exception(f"FTP configuration for {self.name} not found")
        ftp_conf = getattr(ftp_conf, self.name)
        self.ftpserver = FtpServer(serve_path,ftp_conf.ip,ftp_conf.port)

class TransportFTPClient(object):
    def __init__(self, name: str) -> None:
        RasConfigLoader.init()
        self.name = name
        ftp_conf = RasConfigLoader.ras.transport.ftp
        if not hasattr(ftp_conf, self.name):
            raise Exception(f"FTP configuration for {self.name} not found")
        ftp_conf = getattr(ftp_conf, self.name)
        self.ftpclient = FtpClient(ftp_conf.ip,ftp_conf.port)

class TransportMQTTPublisher(object):
    def __init__(self,topic_name) -> None:
        RasConfigLoader.init()
        mqtt_conf = RasConfigLoader.ras.transport.mqtt_broker
        self.mqttpublisher = MqttPublisher(topic_name,mqtt_conf.ip,mqtt_conf.port)

class TransportMQTTSubscriber(object):
    def __init__(self,topic_name,callback) -> None:
        RasConfigLoader.init()
        mqtt_conf = RasConfigLoader.ras.transport.mqtt_broker
        self.mqttsubscriber = MqttSubscriber(topic_name,mqtt_conf.ip,mqtt_conf.port,callback)

def get_mqtt_broker_command():
    RasConfigLoader.init()
    mqtt_conf = RasConfigLoader.ras.transport.mqtt_broker
    return f"mosquitto -p {mqtt_conf.port}"