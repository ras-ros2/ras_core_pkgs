#!/usr/bin/env python3

"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

import rclpy
import json
import time
from awscrt import io,mqtt
from awsiot import mqtt_connection_builder
from os import path
from ras_logging.ras_logger import RasLogger

class MqttConn(object):
    def __init__(self,rcl_node: rclpy.node,config_path:str=None):
        self.mqtt_conn = None
        self.logger = RasLogger()
        self.is_connected = False

        if isinstance(config_path,type(None)):
            return

        self.connect_ep_from_cfg(config_path)
    
    def connect_ep_from_cfg(self,config_path:str):
        if not path.isfile(config_path):
            raise Exception(f"Config path {config_path} does not exist.")
        
        with open(config_path) as f:
            self.config = json.load(f)
        self.connect_ep()


    def connect_ep(self,force=False):
        if self.is_connected and not force:
            self.logger.log_info("Already connected. Skipping.")
            return
        self.mqtt_conn = mqtt_connection_builder.mtls_from_path(
            endpoint=self.config["endpoint"],
            port= self.config["port"],
            cert_filepath= self.config["certificatePath"],
            pri_key_filepath= self.config["privateKeyPath"],
            ca_filepath= self.config["rootCAPath"],
            client_id= self.config["clientID"],
            http_proxy_options=None
        )
        connection_future = self.mqtt_conn.connect()
        res = connection_future.result()
        self.is_connected = True
        self.logger.log_info("Successfully connected.")
    
    def set_last_will(self,topic:str,msg:str,qos=mqtt.QoS.AT_LEAST_ONCE):
        if not self.is_connected:
            raise Exception("Not connected to AWS IoT. Cannot set last will.")
        self.mqtt_conn.will = mqtt.Will(
            topic=topic,
            payload=msg,
            qos=qos
        )

    def disconnect_ep(self):
        if self.is_connected:
            disconnect_future = self.mqtt_conn.disconnect()
            res = disconnect_future.result()
            self.logger.log_info("Successfully disconnected.")
    
    def publish(self,topic,payload,qos=mqtt.QoS.AT_LEAST_ONCE,delay=0.25,wait=False,retry=False):
        if not self.is_connected:
            raise Exception("Not connected to AWS IoT. Cannot publish.")
        
        
        def _publish():
            self.logger.log_info(f"Publishing to {topic}")
            pub_future,id = self.mqtt_conn.publish(
                topic=topic,
                payload=payload,
                qos=qos
            )
            time.sleep(delay)
            if wait:
                res = pub_future.result()
        
        if retry:
            while True:
                try:
                    _publish()
                    break
                except Exception as e:
                    self.logger.log_error(f"Publish failed: {e}. Retrying in {delay} seconds...")
                    time.sleep(delay)
                    self.connect_ep(force=True)
        else:
            _publish()

    def subscribe(self,topic:str,callback,qos=mqtt.QoS.AT_LEAST_ONCE):
        if not self.is_connected:
            raise Exception("Not connected to AWS IoT. Cannot subscribe.")
        assert callable(callback), "The provided callback is not callable."
        sub_future,id = self.mqtt_conn.subscribe(
            topic=topic,
            qos=qos,
            callback=callback
        )
        res = sub_future.result()
        
        

        
