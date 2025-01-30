from setuptools import setup
import os
from glob import glob

package_name = 'floorplan_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'bimpy'],
    package_dir={
        package_name: 'src/' + package_name,
        'bimpy': 'src/' + package_name + '/bimpy/bimpy'
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='armon',
    maintainer_email='armon@todo.todo',
    description='The floorplan_estimation package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'floorplan_estimation_node = floorplan_estimation.floorplan_estimation.floorplan_estimation_node:main',
        ],
    },
)
