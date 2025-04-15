#!/usr/bin/env python3

"""
Copyright (C) 2025 Sachin Kumar

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
Sachin Kumar
Email: info@opensciencestack.org
"""

import os
from .utils import read_yaml
from dataclasses import dataclass, field
from typing import List,Dict
from string import Template
from ras_common.config.loaders.ConfigLoaderBase import ConfigLoaderBase
from ras_resource_lib.generators.common_utils import _expand_template

@dataclass
class PrimitiveTemplate(ConfigLoaderBase):
    type: str
    input_ports: Dict[str, str] = field(default_factory=dict)
    output_ports: Dict[str, str] = field(default_factory=dict)

    def set_name(self, name: str):
        self.name = name

    def prepare_bt_ports(self):
        bt_ports : List[str] = []
        for port in self.input_ports:
            bt_ports.append(f'BT::InputPort<std::string>("{port}")')
        for port in self.output_ports:
            bt_ports.append(f'BT::OutputPort<std::string>("{port}")')

        return ',\n '.join([f'{port}' for port in bt_ports]) 
    def get_instruction_ports(self):
        ports_line = []
        for port in self.input_ports:
            ports_line.append(f"\ti_{port} : str")
        for port in self.output_ports:
            ports_line.append(f"\to_{port} : str")
        return ports_line
    def get_template_data(self) -> dict:
        template_data = {
            "class_name": self.name.capitalize(),
            "provided_ports" : self.prepare_bt_ports(),
        }
        return template_data

# TODO (Sachin) : Will use this dataclass later
# @dataclass
# class PrimitivesConfig(ConfigLoaderBase):
#     primitives: Dict[str, PrimitiveTemplate] = field(default_factory=dict)

class PrimitiveGenerator:
    """
    A class used to generate and manage primitives.

    Attributes
    ----------
        A list of Primitive objects to be generated.

    Methods
    -------
    generate_primitives() -> List[Primitive]
        Generates header files for each primitive in the primitives list.
    load_primitives(path: str) -> bool
        Loads primitives from a YAML file located at the given path.
    _gen_header_file(prim: Primitive)
        Generates a header file for a given primitive using a template.
    """

    def __init__(self, pkg_path: str):
        self.primitives : List[PrimitiveTemplate] = []
        if not self.load_primitives(pkg_path):
            raise ValueError("Error loading primitives")

    def generate_primitives_header_files(self, pkg_path: str) -> None:
        if len(self.primitives) < 1:
            raise ValueError("Primitives not loaded")
        headers_folder = os.path.join(pkg_path,"include")
        gen_headers_folder_name = "gen_primitives"
        generated_headers_folder = os.path.join(headers_folder, gen_headers_folder_name)
        if not os.path.exists(generated_headers_folder):
            os.makedirs(generated_headers_folder)
        templates_path = os.path.join(pkg_path, "templates" )
        template_path = os.path.join(templates_path,"primitive_template.hpp.em")
        generated_files = []
        instruction_lines = []
        for prim in self.primitives:
            instruction_lines.append(f"\n@dataclass\nclass {prim.name}(PrimitiveInstruction):")
            port_lines = prim.get_instruction_ports()
            if len(port_lines)==0:
                instruction_lines.append("\tpass\n")
            else:
                instruction_lines.append("\n".join(port_lines))
            generated_files.append(f"{gen_headers_folder_name}/{prim.name}.hpp")
            _expand_template(template_path, prim.get_template_data(), f"{generated_headers_folder}/{prim.name}.hpp")
        include_template_path = os.path.join(templates_path,"include_primitives.hpp.em")
        _expand_template(include_template_path,{"gen_primitive_files":generated_files},f"{generated_headers_folder}/include_all.hpp")
        primitive_instruction_path = os.path.join(pkg_path,"ras_bt_framework","behaviors","gen_primitives.py")
        primitive_instruction_template = os.path.join(templates_path,"primitive_instructions.py.em")
        _expand_template(primitive_instruction_template,{"instruction_lines":instruction_lines},primitive_instruction_path)
        print(f"Generated header files in {generated_headers_folder}")
    
    def load_primitives(self, pkg_path: str) -> bool:
        config_folder = os.path.join(pkg_path, "config")
        if not os.path.exists(config_folder):
            os.makedirs(config_folder)
        path = os.path.join(config_folder, "primitive_declaration.yaml")
        if "primitives" not in read_yaml(path):
            return False
        data : dict = read_yaml(path)["primitives"]
        for _key, value in data.items():
            prim = PrimitiveTemplate.from_dict(value)
            prim.set_name(_key)
            self.primitives.append(prim)
        return True
