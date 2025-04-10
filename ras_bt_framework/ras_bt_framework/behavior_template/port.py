from dataclasses import dataclass
from abc import ABC, abstractmethod
from ras_common.config.loaders.ConfigLoaderBase import ConfigLoaderBase
from yaml import safe_load

@dataclass
class Port(ABC):
    @abstractmethod
    def serialize(self) -> str:
        pass

@dataclass
class PortEntry(Port):
    name: str
        
    def serialize(self):
        return f"{self.name}"

@dataclass
class PortData(Port):
    @staticmethod
    def default_serialize(value):
        return f"{value}"

class PortDefault(ConfigLoaderBase,PortData):
    pair_delim: str = ';'
    key_val_delim: str = '='
    sep: str = '.'
    @classmethod
    def from_str(cls, value: str):
            flat_dict = {}
            for pair in value.split(cls.pair_delim):
                if cls.key_val_delim in pair:
                    key, value = pair.split(cls.key_val_delim, 1)
                    flat_dict[key] = value
            nested_dict = {}
            for key, value in flat_dict.items():
                keys = key.split(cls.sep)
                d = nested_dict
                for k in keys[:-1]:
                    d = d.setdefault(k, {})
                d[keys[-1]] = safe_load(value)
            return cls.from_dict(nested_dict)
    
    @classmethod
    def to_str(cls, value: object):
        def flatten_dict(d:dict, parent_key=''):
            items = []
            for k, v in d.items():
                new_key = f"{parent_key}{cls.sep}{k}" if parent_key else k
                if isinstance(v, dict):
                    items.extend(flatten_dict(v, new_key).items())
                else:
                    items.append((new_key, v))
            return dict(items)
        flat_dict = flatten_dict(cls.to_dict(value))
        return cls.pair_delim.join(f"{k}{cls.key_val_delim}{v}" for k, v in flat_dict.items())
    
    def serialize(self):
        return self.to_str(self)
            
@dataclass
class PortString(PortData):
    value: str

    def serialize(self):
        return self.value
    
@dataclass
class RefPortEntry(PortEntry):
    name: str
    reference: PortEntry

    def serialize(self):
        PortEntry.serialize(self)

    def ref_serialize(self):
        return f" {self.name}:={self.reference.name} "
