from .common_utils import _create_file,_create_template_file
from pathlib import Path
from ..managers.package_manager import get_asset_package_name,PackageManager
from ..managers.asset_manager import CONFIG_MODULE_NAME
from ..types._base._elements.template import TemplateMap,PythonTemplate
from .cmake_python_gen import generate_pkg
from ..managers.package_manager import AssetType,get_asset_package_name

def populate_template(asset_name,asset_type,package_path,metapkg=False):
    package_name = get_asset_package_name(asset_type,asset_name)
    # generate_pkg(package_name,package_path)
    asset_config = {
        "asset_name":asset_name,
        "project_name":package_name,
    }
    package_path = Path(package_path)
    if metapkg:
        package_path = package_path/package_name
    template_map : TemplateMap = PackageManager.get_module(asset_type,"templates").template_map
    config_template = PythonTemplate(template_map.config)
    asset_config.update(template_map.custom_map)
    generate_pkg(package_name,package_path,asset_config)
    config_template.set_keywords(**asset_config)
    package_name = get_asset_package_name(asset_type,asset_name)
    config_template.create_file(package_path/package_name/package_name/f"{CONFIG_MODULE_NAME}.py")
    for _file,_templ in template_map.file_tree.items():
        _file_path = Path(_file)
        _filename = _file_path.name
        _target_path = package_path/package_name/_file_path.parent
        if not isinstance(_templ,str):
            _templ=""
        if len(_templ)==0:
            _create_file(_filename,_target_path)
        else:
            _pkg_tree = _templ.split("/")
            _templ_name = _pkg_tree.pop()
            module_pkg = f"types.{asset_type.name.lower()}.templates.{'.'.join(_pkg_tree)}"
            _create_template_file(module_pkg,_templ_name,_target_path,_filename,asset_config)
            

        





