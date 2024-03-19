import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'trip_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davidedorigoni',
    maintainer_email='davide.dorigoni@unitn.it',
    description='read/send data from/to the trip low-level-controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trip_base = trip_serial.trip_base:main',
            'trip_unicycle = trip_serial.trip_unicycle:main',
        ],
    },
)
