#!/usr/bin/env python3

import rclpy
from rclpy.exceptions import ParameterNotDeclaredException


def fetch_rosparams():
    params_to_fetch = [
        "host",
        "root_ca_file",
        "certificate_file",
        "private_key_file",
        "port",
        # "topic",
        # "client_id"
    ]

    fetched_params = {}
    node = rclpy.create_node("temp")
    for param in params_to_fetch:
        try:
            value = node.get_parameter(param).get_parameter_value()
            fetched_params[param] = value
        except ParameterNotDeclaredException:
            fetched_params[param] = None
            # Log or handle the missing parameter if necessary

    node.destroy_node()
    return fetched_params

