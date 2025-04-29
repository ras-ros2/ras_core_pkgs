#!/usr/bin/env python3



from ras_bt_framework.generators.primitive_generator.generator import PrimitiveGenerator
from ament_index_python.packages import get_package_share_directory
from ras_common.package.utils import get_cmake_python_pkg_source_dir

def main():
    pkg_share_dir = get_package_share_directory("ras_bt_framework")
    pkg_source_dir = get_cmake_python_pkg_source_dir("ras_bt_framework")
    gen_prim = PrimitiveGenerator(pkg_share_dir)
    gen_prim.generate_primitives_header_files(pkg_source_dir)

if __name__ == '__main__':
    main()