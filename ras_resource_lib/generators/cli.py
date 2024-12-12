import os,argparse
from pathlib import Path
from .cmake_python_gen import generate_pkg
from ..managers.package_manager import AssetType,get_asset_package_name
from .template_gen import populate_template
WORKSPACE_PATH = Path(os.environ["RAS_WORKSPACE_PATH"])


def handle_command(asset_type, asset_name, path=None, metapackage=False):
    if not isinstance(path,str):
        path = WORKSPACE_PATH/'src'/'assets'/asset_type

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
