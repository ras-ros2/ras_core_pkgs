"""
Copyright (C) 2024 Harsh Davda

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

For inquiries or further information, you may contact:
Harsh Davda
Email: info@opensciencestack.org
"""

from xml.etree import ElementTree
from xml.dom import minidom
from pathlib import Path

class BTXml:
    def __init__(self ):
        self.root = ElementTree.Element("root", {"BTCPP_format": "4"})
        self.main_tree = self._new_tree("MainTree")
    
    def set_main_tree(self,id:str):
        self.main_tree = self._new_tree(id)
        
    def _new_tree(self,id:str):
        return ElementTree.SubElement(self.root, "BehaviorTree", {"ID": id})
    
    def new_subtree(self,id:str,subtree_to:ElementTree.Element,port_map):
        new_tree = self._new_tree(id)
        attribs = {"ID": id}
        attribs.update(port_map)
        subtree = ElementTree.SubElement(subtree_to, "SubTree", attribs)
        return (new_tree,subtree)
    
    def add_sequence(self,parent:ElementTree.Element):
        seq = ElementTree.SubElement(parent,"Sequence")
        return seq
    
    def add_script(self,parent:ElementTree.Element,code:str):
        script = ElementTree.SubElement(parent,"Script",{"code": code})
        return script
    
    def add_primitive_node(self,parent:ElementTree.Element,type:str,name:str,port_map:dict):
        attribs = {"name": name} if isinstance(name,str) else {}
        attribs.update(port_map)
        node = ElementTree.SubElement(parent,type,attribs)
        return node

    def dump_xml(self,file_path:str):
        raw_str = ElementTree.tostring(self.root, encoding="utf-8")
        dom = minidom.parseString(raw_str).toprettyxml(indent="\t")
        file_path_h = Path(file_path)
        file_path_h.parent.mkdir(parents=True, exist_ok=True)
        print(dom)
        with file_path_h.open("w") as f:
            f.write(dom)
            f.close()

