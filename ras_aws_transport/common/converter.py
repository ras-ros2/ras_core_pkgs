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


