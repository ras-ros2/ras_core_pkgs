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

from ras_bt_framework.generators.primitive_generator.generator import PrimitiveGenerator
from ament_index_python.packages import get_package_share_directory
from ras_common.package.utils import get_cmake_python_pkg_source_dir

def main():
    pkg_share_dir = get_package_share_directory("ras_bt_framework")
    pkg_source_dir = get_cmake_python_pkg_source_dir("ras_bt_framework")
    gen_prim = PrimitiveGenerator(pkg_share_dir)
    gen_prim.generate_primitives_header_files(pkg_source_dir)

if __name__ == '__main__':
    main()