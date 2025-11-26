from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nano',
    maintainer_email='bryanchang1234@gmail.com',
    description='ROS2 planning package for TM12M pick operations using MoveIt2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_planner = planning.pick_planner:main',
            'simple_move = planning.simple_move:main',
            'pick_and_place = planning.pick_and_place_server:main',
        ],
    },
)
