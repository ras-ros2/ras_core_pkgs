#!/usr/bin/env python3
import rclpy
from ras_bt_framework.generators.behavior_tree_generator import BehaviorTreeGenerator
from ras_bt_framework.managers.primitive_action_manager import PrimitiveActionManager
from ras_bt_framework.behaviors.keywords import TargetPoseMap, rotate, gripper
from ras_bt_framework.behaviors.primitives import SaySomething, ThinkSomethingToSay
from ras_bt_framework.behavior_template.keyword import keyword2module,BehaviorModule,BehaviorModuleSequence
from ras_bt_framework.behaviors.functions import Hello,SaySomethingPy
from ras_bt_framework.behaviors.modules import SaySomething,ThinkSomethingToSay
from ras_bt_framework.managers.BaTMan import BaTMan
import rclpy.node
import os
from ras_common.globals import RAS_APP_PATH,RAS_CONFIGS_PATH
from ras_bt_framework.behavior_utility.yaml_parser import read_yaml_to_pose_dict
from ras_bt_framework.behavior_utility.update_bt import update_xml, update_bt
from ras_common.package.utils import get_cmake_python_pkg_source_dir
from ras_bt_framework.behavior_template.port import PortDefault
from dataclasses import dataclass
def hello():
    print("hello")
    return BehaviorModule()
def test1():
    tpm = TargetPoseMap()
    # kmg = KeywordModuleGenerator()
    tpm.register_pose("pose1","1,2,3,0,0,0,1")
    # kmg.register({
    #     "move2pose":tpm.move2pose_module,
    #     "rotate":rotate,
    #     "gripper":gripper
    # })
    myBehavior = BehaviorModuleSequence()
    myBehavior.add_children([
        keyword2module(tpm.move2pose_module,"move2pose",{"pose":"pose1"}),
        keyword2module(rotate,"rotate",{"angle":90}),
        keyword2module(gripper,"gripper",True),
        SaySomething(i_message="hello bro"),

        # SaySomething(input_ports={"message":"hello"}),
        # Hello(),
        # SaySomethingPy(to_say="hellopy",next_line="this is a test"),
        # SaySomething(input_ports={"message":"bye"}),
        # keyword2module(hello)
    ])
    rclpy.init()
    # node = rclpy.node.Node("my_node")
    # pam = PrimitiveActionManager()
    # my_generator = BehaviorTreeGenerator(pam)
    # my_generator.feed_root(myBehavior)
    # my_generator.verify_sanity()
    # my_generator.generate_xml_trees("test.xml")
    btm = BaTMan()

    path = os.path.join(RAS_CONFIG_PATH,"experiments","0.yaml")
        # print(path)
    # pose_dict,targets = read_yaml_to_pose_dict(path)
    # btm.generate_module_from_keywords(targets,pose_dict)
    btm.run_module("test.xml",myBehavior)
    # new_module = update_bt(btm.main_module)
    # btg = BehaviorTreeGenerator(btm.alfred)
    # pkg_path = get_cmake_python_pkg_source_dir("ras_bt_framework")
    # if pkg_path is None:
    #     btm.get_logger().error("Package Path Not Found")
    #     exit(1)
    
    # bt_path = str(pkg_path)+"/xml/real.xml"
    # btg.feed_root(new_module)
    # try:
    #     btg.generate_xml_trees(bt_path)
    # except Exception as e:
    #     btm.get_logger().error(f"Error in BT Generation: {e}")
    #     exit(1)
    # print(btm.main_module)
    # btm.run_module(myBehavior,"test.xml")
    # aws_pkg_path = get_cmake_python_pkg_source_dir("ras_aws_transport")
    # btm.execute_bt(str(aws_pkg_path)+"/real_bot_zip/real.xml")
    # rclpy.spin(btm)
def test2():
    from ras_bt_framework.behavior_utility.grid_parser import GridConfig
    from ras_common.config.file_utils.formats.yaml import YamlFormat
    from ras_common.globals import RAS_CONFIGS_PATH
    yaml_format = YamlFormat.load(os.path.join(RAS_CONFIGS_PATH,"experiments","test_multi.yaml"))
    for _v in yaml_format["Grids"].values():
        grid = GridConfig.from_dict(_v)
        print(grid)

def main():
    test2()
if __name__ == "__main__":
    main()