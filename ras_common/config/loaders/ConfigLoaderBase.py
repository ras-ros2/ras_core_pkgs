from dataclasses import dataclass, asdict, is_dataclass, fields
from typing import Type, TypeVar, Dict, Any, get_origin,get_type_hints,_BaseGenericAlias as BaseGenericAlias
import dataclasses
import inspect
T = TypeVar('T')

@dataclass
class ConfigElem:
    name: str
    type: type
    default: Any
    field: dataclasses.Field

@dataclass
class ConfigLoaderBase:
    @classmethod
    def load_elems(cls: Type[T]) -> Dict[str, ConfigElem]:
        resolved_hints = get_type_hints(cls)
        field_dict = {field.name:field for field in fields(cls)}
        elems_dict = dict()
        for name,field in field_dict.items():
            field_default = field.default
            if field_default == dataclasses.MISSING:
                field_default = None
            field_type = resolved_hints[name]
            if(isinstance(field_type, BaseGenericAlias)):
                field_type = get_origin(field_type)
            if not isinstance(field_type, type):
                raise Exception(f"Expected type, got {type(field_type)}")
            elems_dict[name] = ConfigElem(name,field_type,field_default, field)
        return elems_dict
    
    @classmethod
    def from_dict(cls: Type[T], config_dict: Dict[str, Any],config_stack: list = []) -> T:
        regen_conf = dict()
        cfg_elems: Dict[str, ConfigElem] = cls.load_elems()
        for k, e in cfg_elems.items():
            if k not in config_dict.keys():
                if isinstance(e.default, type(None)):
                    print(f"{''.join(config_stack)}.{k} not found for {cls.__name__}")
                    exit(1)
                # regen_conf[k] = e.default
                else: 
                    continue
            v = config_dict[k]
            config_stack.append(k)
            if issubclass(e.type, ConfigLoaderBase):
                regen_conf[k] = e.type.from_dict(v,config_stack)
            else:
                if (e.type==float) and isinstance(v,int):
                    regen_conf[k] = e.type(v)
                elif isinstance(v,e.type):
                    regen_conf[k] = v
                else:
                    print(f"Expected {'.'.join(config_stack)} to be {e.type.__name__}, got {type(v).__name__}")
                    exit(1)
            config_stack.pop()
        return cls(**regen_conf)

    @classmethod
    def to_dict(cls: Type[T]) -> Dict[str, Any]:
        cfg_elems: Dict[str, ConfigElem] = cls.load_elems()
        gen_dict = dict()
        for k, e in cfg_elems.items():
            if issubclass(e.type, ConfigLoaderBase):
                gen_dict[k] = e.type.to_dict()
            else:
                if isinstance(e.default, type(None)):
                    sig = inspect.signature(e.type.__init__)
                    has_default = True
                    for _key,_param in sig.parameters.items():
                        # print(_key,_param,str(_param))
                        if _key == 'self':
                            continue
                        if str(_param).startswith('*'):
                            continue
                        if _param.default is inspect.Parameter.empty:
                            has_default = False
                            break
                    if has_default:
                        gen_dict[k] = e.type()
                    else:
                        raise Exception(f"No way to generate default value for {k} of type {e.type}")
                else:
                    gen_dict[k] = e.default
        return gen_dict

