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

