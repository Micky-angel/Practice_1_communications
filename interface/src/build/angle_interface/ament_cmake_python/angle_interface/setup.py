from setuptools import find_packages
from setuptools import setup

setup(
    name='angle_interface',
    version='0.0.0',
    packages=find_packages(
        include=('angle_interface', 'angle_interface.*')),
)
