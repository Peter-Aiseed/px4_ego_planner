from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=find_packages("src"),
    package_dir={"": "src"},
)
setup(**setup_args)