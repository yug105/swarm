from setuptools import find_packages
from setuptools import setup

setup(
    name='px4_msgs_old',
    version='2.0.1',
    packages=find_packages(
        include=('px4_msgs_old', 'px4_msgs_old.*')),
)
