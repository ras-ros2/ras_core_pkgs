from ..config.loaders.ras_config import RasObject as RasConfigLoader,FTPConfig
import os
from pathlib import Path
from .TransportLoader import TransportLoader
from ras_common.globals import RAS_APP_PATH

class TransportFileServer(object):
    serve_path = Path(RAS_APP_PATH)/"serve"
    def __init__(self, name: str) -> None:
        TransportLoader.init()
        RasConfigLoader.init()
        self.serve_path.mkdir(parents=True, exist_ok=True)
        self.name = name
        transport_conf = RasConfigLoader.ras.transport
        file_server = TransportLoader.get_transport(transport_conf.implementation).file_server
        ftp_conf = transport_conf.ftp
        if not hasattr(ftp_conf, self.name):
            raise Exception(f"FTP configuration for {self.name} not found")
        ftp_conf : FTPConfig = getattr(ftp_conf, self.name)
        self.ftpserver = file_server(self.serve_path,ftp_conf.ip,ftp_conf.port)

    def serve(self):
        self.ftpserver.serve()
    
    def connect(self):
        self.ftpserver.connect()

    def safe_kill(self):
        self.ftpserver.safe_kill()

    def __del__(self):
        self.safe_kill()

class TransportFileClient(object):
    def __init__(self, name: str) -> None:
        TransportLoader.init()
        RasConfigLoader.init()
        self.name = name
        file_client = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).file_client
        ftp_conf = RasConfigLoader.ras.transport.ftp
        if not hasattr(ftp_conf, self.name):
            raise Exception(f"FTP configuration for {self.name} not found")
        ftp_conf : FTPConfig = getattr(ftp_conf, self.name)
        self.ftpclient = file_client(ftp_conf.ip,ftp_conf.port)
    
    def connect(self):
        self.ftpclient.connect()
    
    def connect_with_retries(self,delay_sec=5):
        import time
        while True:
            try:
                self.connect()
                print(f"Connected to {self.name} FileServer")
                break
            except Exception as e:
                print(f"Connection to {self.name} FileServer failed: {e}. Retrying in 5 seconds...")
                time.sleep(delay_sec)

    def upload(self, local_path: Path, remote_path: Path):
        self.ftpclient.upload(local_path, remote_path)

    def download(self, remote_path: Path, local_path: Path):
        self.ftpclient.download(remote_path, local_path)
    
    def disconnect(self):
        self.ftpclient.disconnect()
    
    def __del__(self):
        self.disconnect()

class TransportMQTTPublisher(object):
    def __init__(self,topic_name) -> None:
        TransportLoader.init()
        RasConfigLoader.init()
        self.topic_name = topic_name
        mqtt_conf = RasConfigLoader.ras.transport.mqtt
        publisher = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).publisher
        self.mqttpublisher = publisher(topic_name,mqtt_conf.ip,mqtt_conf.port)
    
    def connect(self):
        self.mqttpublisher.connect()
    
    def connect_with_retries(self,delay_sec=5):
        import time
        while True:
            try:
                self.connect()
                print("Connected to MQtt Broker")
                break
            except Exception as e:
                print(f"Connection to MQtt Broker failed: {e}. Retrying in 5 seconds...")
                time.sleep(delay_sec)

    def publish(self, payload : bytes|str|dict):
        import json
        if isinstance(payload,dict):
            payload = json.dumps(payload)
        if isinstance(payload,str):
            payload = payload.encode("utf-8")
        if not isinstance(payload,bytes):
            raise Exception("Payload must be of type bytes, str or dict")
        print(f"Publishing to {self.topic_name}")
        self.mqttpublisher.publish(payload)

    def loop(self):
        self.mqttpublisher.loop()

    def disconnect(self):
        self.mqttpublisher.disconnect()
    
    def __del__(self):
        self.disconnect()

class TransportMQTTSubscriber(object):
    def __init__(self,topic_name,callback) -> None:
        TransportLoader.init()
        RasConfigLoader.init()
        mqtt_conf = RasConfigLoader.ras.transport.mqtt
        subscriber = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).subscriber
        def callback_wrapper(payload):
            print(f"Received message on {topic_name}")
            callback(payload)
        self.mqttsubscriber = subscriber(topic_name,mqtt_conf.ip,mqtt_conf.port,callback_wrapper)
    
    def connect(self):
        self.mqttsubscriber.connect()
    
    def connect_with_retries(self,delay_sec=5):
        import time
        while True:
            try:
                self.connect()
                print("Connected to MQtt Broker")
                break
            except Exception as e:
                print(f"Connection to MQtt Broker failed: {e}. Retrying in 5 seconds...")
                time.sleep(delay_sec)
    
    def loop(self):
        self.mqttsubscriber.loop()
    
    def disconnect(self):
        self.mqttsubscriber.disconnect()

    def __del__(self):
        self.disconnect()

def run_mqtt_broker():
    TransportLoader.init()
    RasConfigLoader.init()
    mqtt_conf = RasConfigLoader.ras.transport.mqtt
    broker_func = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).brocker_func
    broker_func(mqtt_conf.port)