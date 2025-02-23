from abc import ABC,abstractmethod
from pathlib import Path
from typing import Callable
from dataclasses import dataclass,field

class PublisherInterface(ABC):

    @abstractmethod
    def __init__(self, topic: str) -> None:
        pass

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def publish(self,msg: bytes) -> None:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def loop(self) -> None:
        pass

class SubscriberInterface(ABC):

    @abstractmethod
    def __init__(self, topic: str) -> None:
        pass

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def callback(self, msg: bytes) -> None:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass

    @abstractmethod
    def loop(self) -> None:
        pass

class FileServerInterface(ABC):

    @abstractmethod
    def __init__(self, path: Path,ip,port) -> None:
        pass

    def connect(self) -> None:
        pass

    @abstractmethod
    def serve(self) -> None:
        pass
    
    @abstractmethod
    def safe_kill(self) -> None:
        pass

class FileClientInterface(ABC):

    @abstractmethod
    def __init__(self,ip,port) -> None:
        pass

    @abstractmethod
    def connect(self) -> None:
        pass

    @abstractmethod
    def upload(self) -> bool:
        pass
    
    @abstractmethod
    def download(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass


@dataclass
class TransportImplementation:
    name: str
    publisher:type[PublisherInterface]
    subscriber:type[SubscriberInterface]
    file_server:type[FileServerInterface]
    file_client:type[FileClientInterface]
    brocker_func:Callable[[int],None]
