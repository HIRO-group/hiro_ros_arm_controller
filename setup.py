from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hiro_ros_arm_controller'],
    package_dir={'': 'script'}
)
setup(**d)
