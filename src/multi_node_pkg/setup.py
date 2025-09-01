import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multi_node_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguelsilva',
    maintainer_email='miguel.silva@ucb.edu.bo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = multi_node_pkg.sensor_node:main',
            'filter_node = multi_node_pkg.filter_node:main',
            'planner_node = multi_node_pkg.planner_node:main',
            'controller_node = multi_node_pkg.controller_node:main',
            'logger_node = multi_node_pkg.logger_node:main',
        ],
    },
)
