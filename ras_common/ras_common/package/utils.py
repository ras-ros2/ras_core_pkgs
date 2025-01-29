from pathlib import Path
import importlib

def get_cmake_python_pkg_source_dir(package_name: str, no_raise: bool = False) -> Path | None:
    try:
        module_type = importlib.import_module(package_name)
        pkg_path = (Path(module_type.__file__)).resolve().absolute().parent.parent
        return pkg_path
    except Exception as e:
        if not no_raise:
            raise RuntimeError(f"Failed to find the package source directory for '{package_name}': {e}")
        return None
