import shutil,subprocess,os
import ament_copyright
from .common_utils import _create_template_file,_create_file
from ros2pkg.api.create import create_package_environment,populate_ament_cmake,populate_python_libary

def generate_pkg(package_name,dest_dir,extra_config={}):
        from catkin_pkg.package import Person,Package,Dependency,Export
        package_format = 3
        description = 'TODO: Package description'
        license = 'TODO: License declaration'

        available_licenses = {}
        for shortname, entry in ament_copyright.get_licenses().items():
            available_licenses[entry.spdx] = entry.license_files

        maintainer = Person(None)
        git = shutil.which('git')
        if git is not None:
            def get_git_data(data:str):
                p = subprocess.Popen(
                    [git, 'config', f'user.{data}'],
                    stdout=subprocess.PIPE)
                resp = p.communicate()
                return resp[0].decode().rstrip()
            name = get_git_data('name')
            if not name:
                import getpass
                name = getpass.getuser()
            maintainer = Person(name)
            email = get_git_data('email')
            if email:
                maintainer.email = email
        if not maintainer.email:
            maintainer.email = maintainer.name + '@todo.todo'

        buildtool_depends = ['ament_cmake','ament_cmake_ros','ament_cmake_python']
        test_dependencies = ['ament_lint_auto', 'ament_lint_common']
        package = Package(
            package_format=package_format,
            name=package_name,
            version='0.0.0',
            maintainers=[maintainer],
            licenses=[license],
            description= description,
            buildtool_depends=[Dependency(dep) for dep in buildtool_depends],
            test_depends=[Dependency(dep) for dep in test_dependencies],
            exports=[Export('build_type', content='ament_cmake')]
        )

        package_path = os.path.join(dest_dir, package.name)
        if os.path.exists(package_path):
            return '\nAborted!\nThe directory already exists: ' + package_path + '\nEither ' + \
                'remove the directory or choose a different destination directory or package name'

        print('going to create a new package')
        print('package name:', package.name)
        print('destination directory:', os.path.abspath(dest_dir))
        print('package format:', package.package_format)
        print('version:', package.version)
        print('description:', package.description)
        print('maintainer:', [str(maintainer) for maintainer in package.maintainers])
        print('licenses:', package.licenses)
        print('build type:', package.get_build_type())
        print('dependencies:', [str(dependency) for dependency in package.build_depends])
        # if node_name:
        #     print('node_name:', node_name)
        # if library_name:
        #     print('library_name:', library_name)

        package_directory, source_directory, include_directory = \
            create_package_environment(package, dest_dir)
        if not package_directory:
            return 'unable to create folder: ' + dest_dir


        # populate_ament_cmake(package, package_directory, None, None)
        cmakelists_config = {
            'project_name': package.name,
            'dependencies': [str(dep) for dep in package.build_depends],
            'cpp_node_name': None,
            'cpp_library_name': None,
            "env_hooks": False
        }
        cmakelists_config.update(extra_config)
        _create_template_file(
            'generators.templates',
            'CMakeLists.txt',
            package_directory,
            'CMakeLists.txt',
            cmakelists_config)
        _create_file("__init__.py",package_directory+"/"+package_name)

# generate_pkg("test_pkg",".")