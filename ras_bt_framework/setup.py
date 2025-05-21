from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ras_bt_framework'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aether',
    maintainer_email='harsh.davda11@outlook.com',
    description='TODO: Package description',
    license='AGPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
) 