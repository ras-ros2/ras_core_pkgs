from pathlib import Path
import importlib

def get_cmake_python_pkg_source_dir(package_name:str):
    try:
        module_type = importlib.import_module(package_name)
        pkg_path = (Path(module_type.__file__)/"../..").resolve().absolute()
        return pkg_path
    except:
        return None
    