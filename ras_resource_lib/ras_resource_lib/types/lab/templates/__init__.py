from .config import template_func
from ..._base._elements.template import TemplateMap

template_map = TemplateMap(config=template_func,
                           custom_map={"share_paths": ["worlds","models"],
                                       "env_hooks": True},
                           file_tree={"worlds/lab.sdf": "",
                                      "models/.keep": "",
                                    #   "env-hooks/shell_hooks.dsv.in": "env-hooks/shell_hooks.dsv.in",
                                      "env-hooks/shell_hooks.sh.in": "env-hooks/shell_hooks.sh.in"})