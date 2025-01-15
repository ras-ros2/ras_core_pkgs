from dataclasses import dataclass,field
from .TrasnportInterfaces import TransportImplementation
from typing import List,Dict
from pathlib import Path

class TransportLoader(object):
    _initialized = False
    transports:Dict[str,TransportImplementation] = dict()
    @classmethod
    def init(cls):
        if cls._initialized:
            return
        cls._initialized = True

    @classmethod
    def register_transport(cls,transport:TransportImplementation):
        cls.transports[transport.name] = transport

    @classmethod
    def get_transport(cls,name:str)->TransportImplementation:
        if name not in cls.transports:
            raise ValueError(f"Transport {name} not found")
        return cls.transports[name]


from .implementations.DefaultTransport import default_transport

TransportLoader.register_transport(default_transport)
