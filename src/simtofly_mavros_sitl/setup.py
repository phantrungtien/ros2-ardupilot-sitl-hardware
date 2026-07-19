from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simtofly_mavros_sitl'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phantrungtien',
    maintainer_email='phantrungtien1034@gmail.com',
    description='MAVROS configuration for ArduPilot SITL simulation with ROS2 Humble',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_simple = simtofly_mavros_sitl.mission_simple:main',
            'trackinghuman = simtofly_mavros_sitl.trackinghuman:main',
            'verify_frame = simtofly_mavros_sitl.verify_frame:main',
            'mission_circle = simtofly_mavros_sitl.mission_circle:main',
            'mission_gates = simtofly_mavros_sitl.mission_gates:main',
        ],
    },
)