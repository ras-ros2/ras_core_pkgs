import importlib.resources as importlib_resources
from io import StringIO
import sys,em
from pathlib import Path

def _expand_template(template_file, data, output_file):
    output = StringIO()
    interpreter = em.Interpreter(
        output=output,
        options={
            em.BUFFERED_OPT: True,
            em.RAW_OPT: True,
        },
        globals=data,
    )
    output_file = Path(output_file)
    with open(template_file, 'r') as h:
        try:
            interpreter.file(h)
            content = output.getvalue()
        except Exception as e:
            if output_file.is_file():
                output_file.unlink()
            print("Exception when expanding '%s' into '%s': %s" %
                  (template_file, str(output_file), e), file=sys.stderr)
            raise
        finally:
            interpreter.shutdown()

    if (output_file.is_file()):
        with output_file.open("r") as h:
            if h.read() == content:
                return
    else:
        output_file.parent.mkdir(parents=True,exist_ok=True)

    with output_file.open("w") as h:
        h.write(content)
        
def _create_folder(folder_name:str, base_directory:str|Path, exist_ok=True):
    base_directory = Path(base_directory)
    folder_path = base_directory/folder_name

    print('creating folder', folder_path)
    folder_path.mkdir(parents=True,exist_ok=True)
    return str(folder_path)

def _create_file(file_name, base_directory, exist_ok=True):
    _create_template_file("generators.templates","empty",base_directory,file_name,dict())

def _create_template_file(
    submodule_package, template_name, output_directory, output_file_name, template_config
):
    with importlib_resources.path(f"oss_resource_lib.{submodule_package}", template_name+".em") as path:
        template_path = Path(path)
    output_directory = Path(output_directory)
    
    if not template_path.is_file():
        raise FileNotFoundError('template not found:', template_path)

    output_file_path = output_directory/output_file_name

    print('creating', output_file_path)
    _expand_template(str(template_path), template_config, str(output_file_path))
