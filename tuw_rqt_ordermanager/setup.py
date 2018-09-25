# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['tuw_rqt_ordermanager'],
    package_dir={'': 'src'},
    requires=['nav_msgs', 'std_msgs', 'roscpp']
)

setup(**setup_args)
