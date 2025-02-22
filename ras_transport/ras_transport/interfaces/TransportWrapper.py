from ras_common.config.loaders.ras_config import RasObject as RasConfigLoader,FileTransportCfg
import os
from pathlib import Path
from .TransportLoader import TransportLoader
from ras_common.globals import RAS_APP_PATH,RAS_APP_NAME
from threading import Thread
import queue
import time


class TransportFileServer(object):
    serve_path = Path(RAS_APP_PATH)/"serve"
    def __init__(self) -> None:
        TransportLoader.init()
        RasConfigLoader.init()
        self.serve_path.mkdir(parents=True, exist_ok=True)
        transport_conf = RasConfigLoader.ras.transport
        file_server = TransportLoader.get_transport(transport_conf.implementation).file_server
        file_conf = transport_conf.file_server
        self.file_server = file_server(self.serve_path,file_conf.ip,file_conf.port)

    def serve(self):
        self.file_server.serve()
    
    def connect(self):
        self.file_server.connect()

    def safe_kill(self):
        self.file_server.safe_kill()

    def __del__(self):
        self.safe_kill()

class TransportFileClient(object):
    def __init__(self) -> None:
        TransportLoader.init()
        RasConfigLoader.init()
        file_client = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).file_client
        file_conf = RasConfigLoader.ras.transport.file_server
        self.file_client = file_client(file_conf.ip,file_conf.port)
    
    def connect(self):
        self.file_client.connect()
    
    def connect_with_retries(self,delay_sec=5):
        import time
        while True:
            try:
                self.connect()
                print("Connected to FileServer")
                break
            except Exception as e:
                print(f"Connection to FileServer failed: {e}. Retrying in 5 seconds...")
                time.sleep(delay_sec)

    def upload(self, local_path: Path, remote_path: Path):
        self.file_client.upload(local_path, remote_path)

    def download(self, remote_path: Path, local_path: Path):
        self.file_client.download(remote_path, local_path)
    
    def disconnect(self):
        self.file_client.disconnect()
    
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
    def __init__(self,topic_name,callback,queue_size=None) -> None:
        TransportLoader.init()
        RasConfigLoader.init()

        mqtt_conf = RasConfigLoader.ras.transport.mqtt
        subscriber = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).subscriber
        if not isinstance(queue_size,int):
            queue_size=0
        self.cb_queue = queue.Queue(maxsize=queue_size)
        def callback_wrapper(payload):
            print(f"Received message on {topic_name}")
            t = Thread(target=callback,args=(payload,))
            self.cb_queue.put(t)
            # callback(payload)
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
        try:
            thread : Thread = self.cb_queue.get(timeout=1)
            thread.start()
            thread.join(timeout=1)
            if not thread.is_alive():
                del thread
        except queue.Empty:
            pass
    
    def disconnect(self):
        self.mqttsubscriber.disconnect()

    def __del__(self):
        self.disconnect()

class TransportServiceServer(object):
    """
    Represents a service that can be called over MQTT transport.

    Args:
        callback (Callable[[bytes], bytes]): A callback function that will be called
            whenever a request message is received on the service topic. The
            callback should return a response message.
    """
    def __init__(self,topic_name:str,callback,queue_size=None):
        assert callable(callback)
        self.callback = callback
        self.subscriber = TransportMQTTSubscriber("req/"+topic_name,callback=self.srv_callback,queue_size=queue_size)
        self.resp_pub = TransportMQTTPublisher("resp/"+topic_name)
    
    def connect(self):
        self.subscriber.connect()
        self.resp_pub.connect()
        
    def connect_with_retries(self,delay_sec=5):
        self.subscriber.connect_with_retries(delay_sec)
        self.resp_pub.connect_with_retries(delay_sec)
        
    def loop(self):
        self.subscriber.loop()
        self.resp_pub.loop()
        
    def srv_callback(self,payload):
        resp = self.callback(payload)
        if isinstance(resp,str):
            resp = resp.encode("utf-8")
        elif isinstance(resp,type(None)):
            resp = "".encode("utf-8")
        else:
            raise Exception("Response must be of type bytes or str")
        self.resp_pub.publish(resp)
        
    def disconnect(self):
        self.subscriber.disconnect()
        self.resp_pub.disconnect()
        
    def __del__(self):
        self.disconnect()
        
class TransportServiceClient(object):
    """
    Represents a client that can be called over MQTT transport.

    Args:
        topic_name: str
    """
    def __init__(self, topic_name: str):
        assert isinstance(topic_name, str)
        self.req_pub = TransportMQTTPublisher("req/" + topic_name)
        self.subscriber = TransportMQTTSubscriber("resp/" + topic_name, callback=self.on_response)
        self.resp_flag : bool = False
        self.resp = None
        
    def connect(self):
        self.req_pub.connect()
        self.subscriber.connect()
        
    def connect_with_retries(self, delay_sec=5):
        self.req_pub.connect_with_retries(delay_sec)
        self.subscriber.connect_with_retries(delay_sec)
        
    def call(self, payload: bytes | str):
        """
        Default synchronous call.

        Args:
            payload (bytes | str): The payload to send in the service call.
            timeout (int, optional): The timeout for the service call in seconds. Defaults to 10.
        Returns:
            The response from the service call if successful.
        """
        if isinstance(payload, str):
            payload = payload.encode("utf-8")
        elif not isinstance(payload, bytes):
            raise Exception("Payload must be of type bytes or str")
        
        self.req_pub.publish(payload)
        
        while self.resp is None:
            self.loop()
            time.sleep(0.1)
        
        resp = self.resp
        self.resp = None
        return resp
            
    def on_response(self, payload):
        self.resp = payload
    
    def loop(self):
        self.req_pub.loop()
        self.subscriber.loop()
    
    def disconnect(self):
        self.subscriber.disconnect()
        self.req_pub.disconnect()
    
    def __del__(self):
        self.disconnect()

def run_mqtt_broker():
    TransportLoader.init()
    RasConfigLoader.init()
    mqtt_conf = RasConfigLoader.ras.transport.mqtt
    broker_func = TransportLoader.get_transport(RasConfigLoader.ras.transport.implementation).brocker_func
    broker_func(mqtt_conf.port)