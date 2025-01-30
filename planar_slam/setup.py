from setuptools import setup
import os
from glob import glob

package_name = 'planar_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'occamsam'],
    package_dir={
        package_name: 'src/' + package_name,
        'occamsam': 'src/' + package_name + '/occamsam/occamsam'
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
    description='The planar_slam package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planar_slam_node = planar_slam.planar_slam.planar_slam_node:main',
        ],
    },
)
