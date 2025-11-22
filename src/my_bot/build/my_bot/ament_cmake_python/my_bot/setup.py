from setuptools import find_packages
from setuptools import setup

setup(
    name='my_bot',
    version='0.0.0',
    packages=find_packages(
        include=('my_bot', 'my_bot.*')),
)
