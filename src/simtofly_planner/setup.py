from setuptools import find_packages, setup

package_name = 'simtofly_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phantrungtien',
    maintainer_email='phantrungtien1034@gmail.com',
    description='N3 voxel occupancy mapper + N4 Dijkstra path planner for the '
                'gates course.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gate_planner = simtofly_planner.gate_planner:main',
            'plan_overlay = simtofly_planner.plan_overlay:main',
        ],
    },
)
