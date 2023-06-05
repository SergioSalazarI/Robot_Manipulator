from setuptools import find_packages
from setuptools import setup

setup(
    name='servicios',
    version='0.0.0',
    packages=find_packages(
        include=('servicios', 'servicios.*')),
)
