
from ..behavior_template.keyword import KeywordInput,keyword2module
from typing import Dict,Iterable,List,Callable
from ..behavior_template.module import BehaviorModuleSequence,BehaviorModule
from ..behavior_template.instruction import FunctionalInstruction
from ..behaviors.keywords import keyword_mapping
import inspect


class KeywordModuleGenerator(object):
    def __init__(self,default_keywords:Dict[str,Callable]=keyword_mapping):
        self.registered_keywords : Dict[str,type[BehaviorModule]] = default_keywords

    def register(self,keyword: Callable|Dict[str,Callable] , name:str=None):
        if isinstance(keyword,dict):
            for _n,_k in keyword.items():
                self.register(_k,_n)
        elif callable(keyword):
            kw_sig = inspect.signature(keyword)
            for param in kw_sig.parameters.values():
                if str(param).startswith('*'):
                    raise ValueError(f"keyword {keyword} expects an invalid ambiguous parameter {param}") 
            if not isinstance(name,str):
                name = keyword.__name__
            self.registered_keywords[name] = keyword
        else:
            raise ValueError(f"Invalid keyword type: {keyword} {type(keyword)}")

    def generate(self,name,keyword_seq:List[Dict[str,dict]]):
        behavior_module : BehaviorModuleSequence= type(name,(BehaviorModuleSequence,),{})()
        self.generate_into(behavior_module, name, keyword_seq)
        return behavior_module
        
    def generate_into(self, behavior_module: BehaviorModuleSequence, name: str, keyword_seq: List[Dict[str,dict]]):
        """
        Generate keyword modules and add them to an existing BehaviorModuleSequence.
        
        Args:
            behavior_module (BehaviorModuleSequence): The existing module sequence to add to
            name (str): Name of the generated module
            keyword_seq (List[Dict[str,dict]]): Sequence of keywords to process
            
        Returns:
            BehaviorModuleSequence: The updated behavior module sequence
            
        Raises:
            ValueError: If any of the keywords are invalid or unknown
        """
        for _kw in keyword_seq:
            identifier = None
            params = None
            if isinstance(_kw,str):
                identifier = _kw
                params = None
            elif isinstance(_kw,dict):
                if len(_kw)!=1:
                    raise ValueError(f"Invalid keyword input: {_kw}. Expected a single key-value pair")
                identifier,params = list(_kw.items())[0]
            else:
                raise ValueError(f"Invalid keyword input: {_kw}. Expected a single key-value pair")
            if identifier not in self.registered_keywords:
                raise ValueError(f"Unknown keyword: {identifier}")
            keyword = self.registered_keywords[identifier]
            kw_module = keyword2module(keyword,identifier,params)
            if not isinstance(kw_module,BehaviorModule):
                raise ValueError(f"Invalid keyword module: {kw_module}")
            behavior_module.add_children(kw_module)
        return behavior_module
