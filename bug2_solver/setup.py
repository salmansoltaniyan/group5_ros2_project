import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bug2_solver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salman',
    maintainer_email='salman@todo.todo',
    description='Simple Bug2 maze solver — IR + Odometry only',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'bug2_node = bug2_solver.bug2_node:main',
            'bug2_lidar_node = bug2_solver.bug2_lidar_node:main',
        ],
    },
)

