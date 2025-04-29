#!/usr/bin/env python3

import rclpy
import json
from rclpy.serialization import serialize_message,deserialize_message
from collections import OrderedDict
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

def convert_msg_to_json(msg):
    msg_dict : OrderedDict = message_to_ordereddict(msg)
    return json.dumps(msg_dict)

def convert_json_to_msg(json_obj:dict|str, msg_type: type):
    assert isinstance(msg_type, type)
    if isinstance(json_obj,str):
        json_obj = json.loads(json_obj)
    msg = msg_type()
    for field in json_obj.keys():
        assert hasattr(msg, field), f"Attribute '{field}' not found in message type {msg_type.__name__}"
    set_message_fields(msg,json_obj)
    return msg
    
def convert_msg_to_byte_array(msg):
    byte_array = serialize_message(msg)
    return byte_array

def convert_byte_array_to_msg(byte_array, msg_type):
    assert isinstance(msg_type, type)
    msg = deserialize_message(byte_array,msg_type)
    return msg


