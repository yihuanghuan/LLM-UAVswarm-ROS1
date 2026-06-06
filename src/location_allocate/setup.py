# ROS1 catkin setup.py for location_allocate
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['location_allocate'],
    package_dir={'': 'src'},
    scripts=['scripts/location_allocate_node'],
)

setup(**d)
