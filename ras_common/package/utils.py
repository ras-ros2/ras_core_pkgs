from pathlib import Path
import importlib

def get_cmake_python_pkg_source_dir(package_name: str, raise_exception: bool = False) -> Path | None:
    try:
        module_type = importlib.import_module(package_name)
        pkg_path = (Path(module_type.__file__).parent.parent).resolve().absolute()
        return pkg_path
    except Exception as e:
        if raise_exception:
            raise RuntimeError(f"Failed to find the package source directory for '{package_name}': {e}")
        return None
