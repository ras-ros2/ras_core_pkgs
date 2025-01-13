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

import os,argparse
from pathlib import Path
from .cmake_python_gen import generate_pkg
from ..managers.package_manager import AssetType,get_asset_package_name
from .template_gen import populate_template
from ras_common.globals import RAS_WORKSPACE_PATH


def handle_command(asset_type, asset_name, path=None, metapackage=False):
    if not isinstance(path,str):
        path = Path(RAS_WORKSPACE_PATH)/'src'/'assets'/asset_type

    populate_template(asset_name,AssetType[asset_type.upper()],path,metapackage)

def parse_arguments():
    parser = argparse.ArgumentParser(description="Command line interface for generating assets.")
    parser.add_argument("asset_type", type=str, choices=[_typename.lower() for _typename in AssetType._member_names_ if _typename != "NONE"], help="Type of the asset")
    parser.add_argument("asset_name", type=str, help="Name of the asset")
    parser.add_argument("path", type=str, nargs='?', default=None, help="Optional path")
    parser.add_argument("-m","--metapackage", action="store_true", help="Create as a metapackage")
    return parser.parse_args()

def main():
    args = parse_arguments()
    handle_command(args.asset_type, args.asset_name, args.path,args.metapackage)

if __name__ == "__main__":
    main()
