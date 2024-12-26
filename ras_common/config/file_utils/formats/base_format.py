from abc import ABC, abstractmethod
from pathlib import Path

class BaseFormat(ABC):
    @abstractmethod
    def load(self, file_path: Path):
        pass

    @abstractmethod
    def dump(self, data: dict, file_path: Path):
        pass