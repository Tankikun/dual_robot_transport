import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tankikun',
    maintainer_email='tankikun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sync_node = dual_robot_control.sync_node:main',
	    'splitter = dual_robot_control.splitter:main',
        'smart_splitter = dual_robot_control.smart_splitter:main',
        ],
    },
)
