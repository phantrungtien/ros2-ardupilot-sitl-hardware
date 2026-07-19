from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simtofly_gates'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='phantrungtien',
    maintainer_email='phantrungtien1034@gmail.com',
    description='The gate-flythrough stack, N1-N7: find the rings on the camera, '
                'map them, plan through them, fly the plan, judge each pass, '
                'and draw the result back onto the image.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # N1-N2 perception
            'gate_detect = simtofly_gates.gate_detect:main',
            'gate_tracker = simtofly_gates.gate_tracker:main',
            # N3-N4 mapping and planning
            'gate_planner = simtofly_gates.gate_planner:main',
            # N5 trajectory, N6 mission
            'trajectory_follower = simtofly_gates.trajectory_follower:main',
            'mission_fsm = simtofly_gates.mission_fsm:main',
            # N7 overlay
            'plan_overlay = simtofly_gates.plan_overlay:main',
            # stage graders and the stand-still vision check
            'hold_and_watch = simtofly_gates.hold_and_watch:main',
            'verify_n5 = simtofly_gates.verify_n5:main',
            'verify_n6 = simtofly_gates.verify_n6:main',
        ],
    },
)
