from .base_format import BaseFormat
import yaml
from pathlib import Path


class YamlFormat(BaseFormat):
    @staticmethod
    def load( file_path: Path):
        if isinstance(file_path, str):
            file_path = Path(file_path)
        return yaml.safe_load(file_path.open('r'))
    
    @staticmethod
    def dump(data:dict, file_path:Path):
        if isinstance(file_path, str):
            file_path = Path(file_path)
        yaml.safe_dump(data, file_path.open('w'))