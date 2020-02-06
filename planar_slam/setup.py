## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
        package_dir={
            'planar_slam' : 'src/planar_slam',
            'occamsam' : 'src/occamsam/occamsam'}, 
        packages=['planar_slam', 'occamsam']
        )

setup(**setup_args)

